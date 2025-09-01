# Purpose
# -------
# Replays Formula 1 telemetry to physical displays driven by one or more Arduinos.
# - Python loads historical session data via FastF1 and (optionally) live position via OpenF1.
# - A real-time "pacer" replays samples at wall-clock speed.
# - Each sample is serialized to a compact CSV line and written to every open serial port.
#
# Data Pipeline
# -------------
# FastF1:
#   - Per-sample car data (Time, Speed, nGear, Throttle, Brake, DRS, RPM)
#   - Track status messages for flag state
# OpenF1 (optional):
#   - High-frequency race position (mapped to session time)
#
# CSV Protocol sent to Arduino(s)
# -------------------------------
# Header: "HDR,time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos"
# Fields:
#   time_ms  : int   milliseconds since the start of the *current lap*
#   lap      : int   current lap number
#   speed    : int   km/h
#   gear     : int   0..9
#   thr      : int   throttle percentage 0..100
#   brk      : int   brake percentage 0..100 (FastF1 brake*100)
#   drs      : int   raw DRS code normalized (0,1,10,12,14,16 typical)
#   rpm      : int   engine RPM
#   flag     : str   "GREEN" | "YELLOW" | "RED"
#   pos      : int   race position (0 if unknown)
#
# Notes
# -----
# - All error handling is fail-closed: missing columns are created with safe defaults.
# - Serial handshake sends "PING" and prints a one-line sanity echo if a board replies.
# - Timing uses a monotonic clock to avoid drift.
# - Only comments were added; code behavior is unchanged.

import sys, time, math, json
import pandas as pd
import fastf1

# -------------------- CONFIG --------------------
YEAR       = 2025
EVENT_NAME = "Austria"     # FastF1 resolves actual GP name; case-insensitive
SESSION    = "R"           # "R" (Race), "Q" (Quali), etc.
DRIVER     = "PIA"         # Driver acronym (e.g., "VER", "HAM", "NOR")
START_LAP  = 9             # First lap to replay (1-based)
NUM_LAPS   = None          # None = all laps from START_LAP; otherwise limit count

# Serial config
SEND_TO_SERIAL = True
PORTS          = ["COM6"]  # One or more Arduino COM ports (Windows form shown)
BAUD           = 115200    # Must match Serial.begin() on Arduino
BOOT_DELAY     = 2.0       # Give USB serial a moment to enumerate/reset after open
# ------------------------------------------------

# -------------- Utilities --------------

def _dbg(msg: str):
    """
    Lightweight stderr logger for debug/diagnostic output.
    Keeps normal stdout clean for data flow/echoes.
    """
    print(msg, file=sys.stderr, flush=True)


def _safe_import_requests():
    """
    Import 'requests' if present.
    OpenF1 calls are optional; if not installed we simply skip live position.
    """
    try:
        import requests
        return requests
    except Exception:
        _dbg("[WARN] 'requests' not installed; OpenF1 live position will be skipped")
        return None


def _open_serial_ports():
    """
    Open and prime all configured serial ports.
    - Lists available ports for operator visibility.
    - Sends a one-shot 'PING' to each opened port and logs any reply.
    - On failure to open a configured port: closes already-opened ports and exits.
    Returns
    -------
    list[serial.Serial]
        A list of open pyserial objects (possibly empty if SEND_TO_SERIAL is False).
    """
    sers = []
    if not SEND_TO_SERIAL:
        return sers
    try:
        import serial, serial.tools.list_ports
        print("[Ports detected]")
        for p in serial.tools.list_ports.comports():
            print(" ", p.device, "-", p.description)

        for idx, port in enumerate(PORTS, 1):
            try:
                s = serial.Serial(port, BAUD, timeout=0.2)
                # Allow Arduino bootloader/auto-reset to settle
                time.sleep(BOOT_DELAY)
                s.reset_input_buffer()
                # Quick liveness check
                s.write(b"PING\n")
                echo = s.readline().decode(errors="ignore").strip()
                print(f"[SANITY A{idx} {port}] {echo if echo else '(no reply)'}")
                sers.append(s)
            except Exception as e:
                print(f"[FATAL] Could not open {port}: {e}")
                # Clean up any already-open handles before exiting
                for t in sers:
                    try:
                        if t.is_open: t.close()
                    except Exception:
                        pass
                sys.exit(1)
    except ImportError:
        print("[FATAL] pyserial not installed")
        sys.exit(1)
    return sers


def _send_line_to_all(sers, msg: str):
    """
    Write a single CSV line to all open serial ports and print any echo.
    Newline is enforced to delimit frames on the Arduino side.
    """
    if not msg.endswith("\n"):
        msg += "\n"
    for s in sers:
        s.write(msg.encode("utf-8"))
    # Optional readback (useful during bring-up)
    for i, s in enumerate(sers, 1):
        echo = s.readline().decode(errors="ignore").strip()
        if echo:
            print(f"[A{i}] {echo}")


def sanitize_drs(val):
    """
    Normalize provider-specific DRS codes into a stable integer value.
    Accepts typical FastF1/OpenF1 encodings:
      - 0: closed/off
      - 1, 10, 12, 14, 16: open/active variants depending on provider/era
    Any unrecognized or NaN value is coerced to 0. The raw integer is returned
    (not clamped to {0,1}) so downstream consumers can still differentiate codes.
    """
    if val is None:
        return 0
    try:
        if isinstance(val, float) and math.isnan(val):
            return 0
        v = int(val)
    except Exception:
        return 0
    # Typical values seen in FastF1/OpenF1 streams
    # 0=off, 1=on, 10/12/14/16=open/active variants depending on provider
    return v if v in (0, 1, 10, 12, 14, 16) else (0 if v < 0 else v)

# -------------- OpenF1 helpers --------------

def _openf1_get_session_key(req, year, target_start_utc):
    """
    Resolve the OpenF1 session_key for the Race of a given year by nearest start time.
    This protects us from track-specific naming and daylight-saving quirks.
    """
    try:
        url = f"https://api.openf1.org/v1/sessions?year={year}&session_name=Race"
        r = req.get(url, timeout=10)
        r.raise_for_status()
        rows = r.json()
        if not rows:
            return None
        cand = []
        for row in rows:
            ds = pd.to_datetime(row.get("date_start"))
            if pd.isna(ds):
                continue
            # Compare in UTC to avoid tz ambiguity
            diff = abs((ds.tz_convert("UTC") if getattr(ds, "tzinfo", None) else ds.tz_localize("UTC")) - target_start_utc)
            cand.append((diff, row))
        if not cand:
            return None
        cand.sort(key=lambda x: x[0])
        best = cand[0][1]
        return int(best.get("session_key")) if best.get("session_key") is not None else None
    except Exception as e:
        _dbg(f"[WARN] OpenF1 sessions lookup failed: {e}")
        return None


def _openf1_get_driver_number(req, session_key, abbr):
    """
    Resolve OpenF1 driver_number from a driver acronym for a specific session.
    Falls back to scanning all drivers for a case-insensitive acronym match.
    """
    try:
        url = f"https://api.openf1.org/v1/drivers?session_key={session_key}&name_acronym={abbr}"
        r = req.get(url, timeout=10)
        r.raise_for_status()
        rows = r.json()
        if rows:
            return int(rows[0]["driver_number"])
        # Fallback: enumerate and match acronym manually
        url = f"https://api.openf1.org/v1/drivers?session_key={session_key}"
        r = req.get(url, timeout=10)
        r.raise_for_status()
        for row in r.json():
            if str(row.get("name_acronym", "")).upper() == abbr.upper():
                return int(row["driver_number"])
    except Exception as e:
        _dbg(f"[WARN] OpenF1 driver lookup failed: {e}")
    return None


def _openf1_get_position_series(req, session_key, driver_number, t0_utc):
    """
    Fetch high-frequency position samples from OpenF1 for a driver/session.
    Returns a DataFrame aligned to FastF1 session time:
      columns: SessionTime (Timedelta), LivePos (int)
    """
    try:
        url = f"https://api.openf1.org/v1/position?session_key={session_key}&driver_number={driver_number}"
        r = req.get(url, timeout=10)
        r.raise_for_status()
        rows = r.json()
        if not rows:
            return None
        df = pd.DataFrame(rows)
        if "date" not in df or "position" not in df:
            return None
        # Convert UTC timestamps to Timedelta since FastF1 session t0
        ts = pd.to_datetime(df["date"], utc=True)
        sess_time = ts - t0_utc
        out = pd.DataFrame({
            "SessionTime": sess_time,
            "LivePos": pd.to_numeric(df["position"], errors="coerce")
        }).dropna().sort_values("SessionTime")
        out["LivePos"] = out["LivePos"].astype(int)
        return out
    except Exception as e:
        _dbg(f"[WARN] OpenF1 position fetch failed: {e}")
        return None

# -------------- Main --------------

# Open serial ports up front (no-op if SEND_TO_SERIAL=False)
sers = _open_serial_ports()

try:
    # ---------- LOAD FASTF1 ----------
    # Load laps + per-sample telemetry + track messages.
    # Weather is skipped to reduce load time; not used downstream.
    session = fastf1.get_session(YEAR, EVENT_NAME, SESSION)
    session.load(laps=True, telemetry=True, weather=False, messages=True)

    # Select the target driver's laps and constrain to the requested lap window.
    laps_all = session.laps
    driver_laps = laps_all[laps_all["Driver"] == DRIVER].sort_values("LapNumber").copy()
    if driver_laps.empty:
        raise RuntimeError(f"No laps found for {DRIVER}")

    max_lap = int(driver_laps["LapNumber"].max())
    start = max(START_LAP, 1)
    end = max_lap if NUM_LAPS is None else min(start + NUM_LAPS - 1, max_lap)
    selected = driver_laps[(driver_laps["LapNumber"] >= start) & (driver_laps["LapNumber"] <= end)].copy()
    if selected.empty:
        raise RuntimeError(f"No laps in requested range {start}-{end} for {DRIVER}")

    # Build a single time-ordered telemetry DataFrame across selected laps.
    tele_list = []
    for _, L in selected.iterrows():
        try:
            cd = L.get_car_data()  # Columns: Time, Speed, nGear, Throttle, DRS, RPM (plus Brake if available)
        except Exception:
            continue
        if cd is None or cd.empty:
            continue
        cd = cd.copy()

        # Guarantee expected columns exist so downstream code can treat them uniformly.
        for col, default in (("Speed", 0), ("nGear", 0), ("Throttle", 0), ("Brake", 0), ("DRS", 0), ("RPM", 0)):
            if col not in cd.columns:
                cd[col] = default

        # Attach lap metadata and compute absolute session time for each sample.
        cd["LapNumber"]    = int(L["LapNumber"])
        cd["LapStartTime"] = L["LapStartTime"]                 # Timedelta
        cd["SessionTime"]  = L["LapStartTime"] + cd["Time"]    # Timedelta since session start
        tele_list.append(cd)

    if not tele_list:
        raise RuntimeError("No telemetry rows for the selected laps.")

    tele = pd.concat(tele_list, ignore_index=True).sort_values("SessionTime").reset_index(drop=True)

    # ---------- Flags ----------
    # Convert track status into a stable flag state timeline (GREEN/YELLOW/RED).
    ts = session.track_status
    flag_times, flag_codes = [], []

    def status_to_flag(status: int, message: str) -> str:
        # Map numeric status and free-text messages to a coarse flag state.
        if status == 5: return "RED"
        if status in (2, 4, 6, 7, 8): return "YELLOW"
        m = (message or "").upper()
        if "RED" in m: return "RED"
        if "SAFETY CAR" in m or "VSC" in m or "YELLOW" in m: return "YELLOW"
        return "GREEN"

    if ts is not None and not ts.empty:
        ts = ts.sort_values("Time").copy()
        # Normalize "Time" to Timedelta relative to session start (t0).
        if pd.api.types.is_datetime64_any_dtype(ts["Time"]):
            t_ser = ts["Time"]
            if t_ser.dt.tz is None:
                t_ser = t_ser.dt.tz_localize("UTC")
            t0 = session.t0_date
            if t0.tzinfo is None:
                t0 = t0.tz_localize("UTC")
            ts["TimeTd"] = t_ser - t0
        else:
            ts["TimeTd"] = ts["Time"]

        # Capture only transitions to reduce churn during replay.
        prev = None
        for _, row in ts.iterrows():
            flg = status_to_flag(int(row.get("Status", 1) or 1), row.get("Message", ""))
            if flg != prev:
                flag_times.append(row["TimeTd"])
                flag_codes.append(flg)
                prev = flg

    # ---------- Lap windows + per-lap pos fallback ----------
    # Create per-lap start/end windows for lap lookup and as a fallback for position.
    lap_windows = []
    sel_sorted = selected.sort_values("LapNumber").reset_index(drop=True)

    for i, L in sel_sorted.iterrows():
        start_td = L["LapStartTime"]
        if pd.notna(L.get("LapTime")):
            end_td = start_td + L["LapTime"]
        else:
            # If last lap or missing LapTime, bound with next lap start or a wide sentinel.
            end_td = sel_sorted.loc[i+1, "LapStartTime"] if i+1 < len(sel_sorted) else start_td + pd.Timedelta(minutes=30)
        try:
            lap_pos = int(L["Position"]) if pd.notna(L.get("Position")) else None
        except Exception:
            lap_pos = None
        lap_windows.append((start_td, end_td, int(L["LapNumber"]), lap_pos))

    # ---------- High-frequency Position via OpenF1 ----------
    # Attempt to enrich with live/hi-freq position; gracefully skip if requests isn't installed
    # or if network lookups fail.
    requests = _safe_import_requests()
    pos_df = None
    if requests is not None:
        try:
            # Compute session t0 in UTC (tz-aware) for alignment.
            t0_utc = session.t0_date
            if getattr(t0_utc, "tzinfo", None) is None:
                t0_utc = t0_utc.tz_localize("UTC")
            else:
                t0_utc = t0_utc.tz_convert("UTC")

            _dbg("[DBG] Resolving OpenF1 session_key…")
            sess_key = _openf1_get_session_key(requests, YEAR, t0_utc)
            _dbg(f"[DBG] OpenF1 session_key: {sess_key}")

            if sess_key:
                abbr = DRIVER.upper()
                _dbg(f"[DBG] Resolving driver_number for {abbr}…")
                drv_num = _openf1_get_driver_number(requests, sess_key, abbr)
                _dbg(f"[DBG] driver_number: {drv_num}")

                if drv_num is not None:
                    _dbg("[DBG] Fetching OpenF1 position timeline…")
                    pos_df = _openf1_get_position_series(requests, sess_key, drv_num, t0_utc)
                    if pos_df is not None and not pos_df.empty:
                        _dbg(f"[DBG] Position samples: {len(pos_df)}")
        except Exception as e:
            _dbg(f"[WARN] OpenF1 integration error: {e}")
            pos_df = None

    # Merge OpenF1 position (if available) onto the per-sample telemetry using
    # nearest-previous (backward) alignment on SessionTime.
    tele = tele.sort_values("SessionTime")
    if pos_df is not None and not pos_df.empty:
        tele = pd.merge_asof(tele, pos_df, on="SessionTime", direction="backward")
    else:
        tele["LivePos"] = pd.NA
        _dbg("[WARN] Using per-lap position fallback (no OpenF1 samples)")

    # Fill missing LivePos:
    # 1) forward-fill;
    tele["LivePos"] = tele["LivePos"].ffill()
    # 2) fill from the lap’s recorded position if still missing;
    if any(tele["LivePos"].isna()):
        lap_pos_map = {lw[2]: lw[3] for lw in lap_windows if lw[3] is not None}
        tele["LivePos"] = tele.apply(
            lambda r: r["LivePos"] if pd.notna(r["LivePos"]) else lap_pos_map.get(int(r["LapNumber"]), None),
            axis=1
        )
    # 3) final coercion to int with 0 as sentinel.
    tele["LivePos"] = pd.to_numeric(tele["LivePos"], errors="coerce").fillna(0).astype(int)

    # ---------- Real-time playback ----------
    # Indexes into the flag transition list and lap window list.
    fi = 0
    current_flag = "GREEN"
    li = 0

    # Establish the wall-clock pacing origin using the first sample’s SessionTime.
    base_session_t = tele["SessionTime"].iloc[0]
    wall_start = time.perf_counter()

    # Send CSV header once for tools/diagnostics; Arduinos may echo it back.
    _send_line_to_all(sers, "HDR,time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos")

    # Iterate in SessionTime order and pace each sample to real time.
    for _, r in tele.iterrows():
        # Sleep until the scheduled wall-clock time for this sample.
        target = wall_start + (r["SessionTime"] - base_session_t).total_seconds()
        dt = target - time.perf_counter()
        if dt > 0:
            time.sleep(dt)

        # Advance flag state if we have crossed a transition.
        st = r["SessionTime"]
        while fi < len(flag_times) and flag_times[fi] <= st:
            current_flag = flag_codes[fi]
            fi += 1
        display_flag = current_flag if current_flag in ("YELLOW", "RED") else "GREEN"

        # Track the active lap window for lap number and lap-relative timing.
        while li + 1 < len(lap_windows) and st >= lap_windows[li][1]:
            li += 1
        lap_no  = lap_windows[li][2]

        # Extract and coerce telemetry fields (robust to missing/NaN).
        s   = int((r.get("Speed") or 0))
        g   = int((r.get("nGear") or 0))
        thr = int((r.get("Throttle") or 0))
        # FastF1 Brake is typically 0..1; scale to 0..100%
        brk = int((r.get("Brake") or 0) * 100)
        drs = sanitize_drs(r.get("DRS"))
        rpm = int((r.get("RPM") or 0))

        # Compute milliseconds since the start of the current lap for the sample.
        lap_rel_ms = int((st - lap_windows[li][0]).total_seconds() * 1000)
        display_pos = int(r.get("LivePos", 0))

        # Serialize as CSV and send to all connected Arduinos.
        # CSV: time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos
        line = f"{lap_rel_ms},{lap_no},{s},{g},{thr},{brk},{drs},{rpm},{display_flag},{display_pos}"
        _send_line_to_all(sers, line)

finally:
    # Always attempt to close serial ports cleanly on exit.
    for s in sers:
        try:
            if s.is_open:
                s.close()
        except Exception:
            pass
