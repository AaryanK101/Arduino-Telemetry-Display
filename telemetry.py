# telemetry_to_two_arduinos.py
import sys, time, math, json
import pandas as pd
import fastf1

# -------------------- CONFIG --------------------
YEAR       = 2025
EVENT_NAME = "austria"     # FastF1 resolves actual GP name
SESSION    = "R"           # "R" (Race), "Q" (Quali), etc.
DRIVER     = "NOR"         # three-letter code
START_LAP  = 1
NUM_LAPS   = None          # None = all laps from START_LAP

# Serial config
SEND_TO_SERIAL = True
PORTS          = ["COM6"]  # set your Arduino COM ports
BAUD           = 115200
BOOT_DELAY     = 2.0
# ------------------------------------------------

# -------------- Utilities --------------
def _dbg(msg: str):
    print(msg, file=sys.stderr, flush=True)

def _safe_import_requests():
    try:
        import requests
        return requests
    except Exception:
        _dbg("[WARN] 'requests' not installed; OpenF1 live position will be skipped")
        return None

def _open_serial_ports():
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
                time.sleep(BOOT_DELAY)
                s.reset_input_buffer()
                s.write(b"PING\n")
                echo = s.readline().decode(errors="ignore").strip()
                print(f"[SANITY A{idx} {port}] {echo if echo else '(no reply)'}")
                sers.append(s)
            except Exception as e:
                print(f"[FATAL] Could not open {port}: {e}")
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
    if not msg.endswith("\n"):
        msg += "\n"
    for s in sers:
        s.write(msg.encode("utf-8"))
    for i, s in enumerate(sers, 1):
        echo = s.readline().decode(errors="ignore").strip()
        if echo:
            print(f"[A{i}] {echo}")

# Normalize DRS to an integer code (keep common Open/Active codes)
def sanitize_drs(val):
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
    try:
        url = f"https://api.openf1.org/v1/drivers?session_key={session_key}&name_acronym={abbr}"
        r = req.get(url, timeout=10)
        r.raise_for_status()
        rows = r.json()
        if rows:
            return int(rows[0]["driver_number"])
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
sers = _open_serial_ports()

try:
    # ---------- LOAD FASTF1 ----------
    session = fastf1.get_session(YEAR, EVENT_NAME, SESSION)
    session.load(laps=True, telemetry=True, weather=False, messages=True)

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

    tele_list = []
    for _, L in selected.iterrows():
        try:
            cd = L.get_car_data()  # Time, Speed, nGear, Throttle, DRS, RPM
        except Exception:
            continue
        if cd is None or cd.empty:
            continue
        cd = cd.copy()

        # Guarantee expected columns exist
        for col, default in (("Speed", 0), ("nGear", 0), ("Throttle", 0), ("Brake", 0), ("DRS", 0), ("RPM", 0)):
            if col not in cd.columns:
                cd[col] = default

        cd["LapNumber"]    = int(L["LapNumber"])
        cd["LapStartTime"] = L["LapStartTime"]                 # Timedelta
        cd["SessionTime"]  = L["LapStartTime"] + cd["Time"]    # Timedelta since session start
        tele_list.append(cd)

    if not tele_list:
        raise RuntimeError("No telemetry rows for the selected laps.")

    tele = pd.concat(tele_list, ignore_index=True).sort_values("SessionTime").reset_index(drop=True)

    # ---------- Flags ----------
    ts = session.track_status
    flag_times, flag_codes = [], []

    def status_to_flag(status: int, message: str) -> str:
        if status == 5: return "RED"
        if status in (2, 4, 6, 7, 8): return "YELLOW"
        m = (message or "").upper()
        if "RED" in m: return "RED"
        if "SAFETY CAR" in m or "VSC" in m or "YELLOW" in m: return "YELLOW"
        return "GREEN"

    if ts is not None and not ts.empty:
        ts = ts.sort_values("Time").copy()
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

        prev = None
        for _, row in ts.iterrows():
            flg = status_to_flag(int(row.get("Status", 1) or 1), row.get("Message", ""))
            if flg != prev:
                flag_times.append(row["TimeTd"])
                flag_codes.append(flg)
                prev = flg

    # ---------- Lap windows + per-lap pos fallback ----------
    lap_windows = []
    sel_sorted = selected.sort_values("LapNumber").reset_index(drop=True)

    for i, L in sel_sorted.iterrows():
        start_td = L["LapStartTime"]
        if pd.notna(L.get("LapTime")):
            end_td = start_td + L["LapTime"]
        else:
            end_td = sel_sorted.loc[i+1, "LapStartTime"] if i+1 < len(sel_sorted) else start_td + pd.Timedelta(minutes=30)
        try:
            lap_pos = int(L["Position"]) if pd.notna(L.get("Position")) else None
        except Exception:
            lap_pos = None
        lap_windows.append((start_td, end_td, int(L["LapNumber"]), lap_pos))

    # ---------- High-frequency Position via OpenF1 ----------
    requests = _safe_import_requests()
    pos_df = None
    if requests is not None:
        try:
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

    tele = tele.sort_values("SessionTime")
    if pos_df is not None and not pos_df.empty:
        tele = pd.merge_asof(tele, pos_df, on="SessionTime", direction="backward")
    else:
        tele["LivePos"] = pd.NA
        _dbg("[WARN] Using per-lap position fallback (no OpenF1 samples)")

    # Fill missing LivePos
    tele["LivePos"] = tele["LivePos"].ffill()
    if any(tele["LivePos"].isna()):
        lap_pos_map = {lw[2]: lw[3] for lw in lap_windows if lw[3] is not None}
        tele["LivePos"] = tele.apply(
            lambda r: r["LivePos"] if pd.notna(r["LivePos"]) else lap_pos_map.get(int(r["LapNumber"]), None),
            axis=1
        )
    tele["LivePos"] = pd.to_numeric(tele["LivePos"], errors="coerce").fillna(0).astype(int)

    # ---------- Real-time playback ----------
    fi = 0
    current_flag = "GREEN"
    li = 0

    base_session_t = tele["SessionTime"].iloc[0]
    wall_start = time.perf_counter()

    _send_line_to_all(sers, "HDR,time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos")

    for _, r in tele.iterrows():
        target = wall_start + (r["SessionTime"] - base_session_t).total_seconds()
        dt = target - time.perf_counter()
        if dt > 0:
            time.sleep(dt)

        st = r["SessionTime"]
        while fi < len(flag_times) and flag_times[fi] <= st:
            current_flag = flag_codes[fi]
            fi += 1
        display_flag = current_flag if current_flag in ("YELLOW", "RED") else "GREEN"

        while li + 1 < len(lap_windows) and st >= lap_windows[li][1]:
            li += 1
        lap_no  = lap_windows[li][2]

        s   = int((r.get("Speed") or 0))
        g   = int((r.get("nGear") or 0))
        thr = int((r.get("Throttle") or 0))
        brk = int((r.get("Brake") or 0) * 100)
        drs = sanitize_drs(r.get("DRS"))
        rpm = int((r.get("RPM") or 0))

        lap_rel_ms = int((st - lap_windows[li][0]).total_seconds() * 1000)
        display_pos = int(r.get("LivePos", 0))

        # CSV: time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos
        line = f"{lap_rel_ms},{lap_no},{s},{g},{thr},{brk},{drs},{rpm},{display_flag},{display_pos}"
        _send_line_to_all(sers, line)

finally:
    for s in sers:
        try:
            if s.is_open:
                s.close()
        except Exception:
            pass
