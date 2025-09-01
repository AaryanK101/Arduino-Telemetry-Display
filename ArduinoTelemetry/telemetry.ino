/***********************************************************
  F1 TELEMETRY DISPLAY (Arduino Side)
  ----------------------------------------------------------
  Purpose
  -------
  Render an F1 telemetry stream on physical modules:
    • TM1637 4-digit display: vehicle speed
    • Single 7-segment: current gear
    • 8×8 LED matrix (MAX7219): throttle and brake bars
    • 16×2 I²C LCD: lap time, delta, RPM, race position
    • LEDs: track flags (R/Y/G) and DRS active indicator

  Data Source & Protocol
  ----------------------
  Python streams CSV lines over USB serial at 115200 baud.
  Header (sent once for visibility):
    HDR,time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos
  Frame example:
    35124,8,267,6,72,00,12,12450,GREEN,5

  Field meanings:
    time_ms : ms since start of the current lap
    lap     : current lap number
    speed   : km/h
    gear    : 0..9
    thr     : throttle percent (0..100)
    brk     : brake percent (0..100)
    drs     : DRS code (0=off; 1/10/12/14/16 treated as active)
    rpm     : engine RPM
    flag    : "GREEN" | "YELLOW" | "RED"
    pos     : race position (0 if unknown)

  Hardware Wiring (Uno)
  ---------------------
  I²C LCD (hd44780 + backpack):  SDA=A4, SCL=A5
  TM1637 (speed):                CLK=D11, DIO=D12
  MAX7219 (8×8):                 DIN=A0, CS=D2, CLK=D3
  Seven-segment (gear):          a=D9, b=D8, c=D4, d=D5, e=D6, f=D10, g=D13
  LEDs:                          DRS=D7, FlagR=A1, FlagY=A2, FlagG=A3

  Display Behavior
  ----------------
  • LCD is updated on a fixed cadence (50 ms) and extrapolates lap time
    between serial frames for smoothness.
  • 8×8 matrix shows two 3-column vertical bars:
        columns 0..2  -> brake (0..100%)
        columns 5..7  -> throttle (0..100%)
    Bars are drawn bottom-up with a small bias so low non-zero values
    light at least one row.
  • DRS indicator drives a single LED high when a known "active" code is seen.

  Notes
  -----
  • All parsing is index-based for speed. If the CSV format changes,
    adjust the indices in the parsing block.
  • Serial echo lines are helpful for bring-up and logging but can be
    disabled if needed.

***********************************************************/

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

// ===== TM1637 speed display =====
#include <TM1637Display.h>
#define TM_CLK 11
#define TM_DIO 12
TM1637Display spdDisplay(TM_CLK, TM_DIO);

// ===== MAX7219 pins (reuse former DRS pins) =====
const int MAX_DIN = A0;   // Data in (DIN)
const int MAX_CLK = 3;    // Clock (CLK)
const int MAX_CS  = 2;    // Chip select / LOAD (CS)

// ---------- LCD (I²C) ----------
hd44780_I2Cexp lcd;

// ---------- Single 7-segment pins (gear) ----------
int a=9, b=8, c=4, d=5, e=6, f=10, g=13;

// ---------- DRS and flag LEDs ----------
const int DRS_PIN   = 7;   // DRS active indicator

const int FLAG_R_PIN = A1; // Red flag LED
const int FLAG_G_PIN = A3; // Green flag LED
const int FLAG_Y_PIN = A2; // Yellow flag LED

// ---------- Telemetry state (latest values) ----------
long   time_ms = 0;
int    lap = 0, speed = 0, gear = 0;
int    thr = 0, brk = 0;      // Percent 0..100
int    drs = 0, rpm = 0;
int    pos = 0;
String flag;

// ---------- Lap timing / delta bookkeeping ----------
int    curLap = -1;           // Tracks the lap we believe we are on
long   lastLapMs    = -1;     // Duration of the last completed lap
long   prevLapMs    = -1;     // Duration of the lap before that
String lastDeltaStr = "   --.-"; // Human-readable delta vs. previous lap

// ---------- Smooth LCD updater (rate-limited) ----------
unsigned long lastTeleWallMs = 0; // Wall-clock timestamp of last telemetry update
long          lastTeleLapMs  = 0; // time_ms at last telemetry update
bool          stream_active  = false; // Whether we have received a frame recently
unsigned long lastRxWallMs   = 0;     // Wall-clock time of last received frame

const unsigned long LCD_PERIOD_MS     = 50;  // Min. interval between LCD updates
const unsigned long STREAM_TIMEOUT_MS = 250; // If we miss frames for this long, freeze extrapolation
unsigned long lastLcdTick = 0;

// =============== setup/loop ===============

void setup(){
  // Configure the discrete 7-segment pins for the gear display
  pinMode(a, OUTPUT); pinMode(b, OUTPUT); pinMode(c, OUTPUT); pinMode(d, OUTPUT);
  pinMode(e, OUTPUT); pinMode(f, OUTPUT); pinMode(g, OUTPUT);

  // LEDs: initialize as outputs and start off
  pinMode(DRS_PIN, OUTPUT);
  pinMode(FLAG_R_PIN, OUTPUT);
  pinMode(FLAG_G_PIN, OUTPUT);
  pinMode(FLAG_Y_PIN, OUTPUT);
  digitalWrite(DRS_PIN, LOW);
  digitalWrite(FLAG_R_PIN, LOW);
  digitalWrite(FLAG_G_PIN, LOW);
  digitalWrite(FLAG_Y_PIN, LOW);

  // LCD bring-up
  Wire.begin();                 // I²C on SDA=A4 / SCL=A5
  int status = lcd.begin(16, 2);
  if (status) hd44780::fatalError(status);
  lcd.backlight();
  lcd.clear();
  printRow16(0, "00:00.000", "");   // Default lap time display
  printRow16(1, "RPM:0", "P:--");   // Default RPM/position display

  // Brief gear segment animation while booting
  digital_0(); delay(120); digital_dash(); delay(120);

  // TM1637 (speed) bring-up
  spdDisplay.setBrightness(0x0F);
  spdDisplay.clear();

  // MAX7219 (8×8 bars) bring-up
  maxInit();
  fbClear(); fbFlush();

  // Serial interface for CSV frames
  Serial.begin(115200);
  while (!Serial) {}            // Wait for USB CDC
  Serial.setTimeout(20);        // Keep reads snappy
  Serial.println("Arduino ready");

  // Initialize runtime state
  digital_dash();
  stream_active  = false;
  lastTeleLapMs  = 0;
  lastTeleWallMs = millis();
  lastRxWallMs   = 0;
}

void loop(){
  // Periodic LCD refresh with extrapolated lap time
  updateLcdSmooth();

  // No data pending on serial: nothing to parse
  if (!Serial.available()) return;

  // Read one CSV line delimited by '\n'
  String msg = Serial.readStringUntil('\n');
  msg.trim();
  if (msg.length() == 0) return;

  // Lightweight command / header handling
  if (msg == "PING") { Serial.println("PONG-A"); return; }  // Host sanity ping
  if (msg.startsWith("HDR")) { Serial.println(msg); return; } // Echo header for logs

  // -------- Parse telemetry CSV by index --------
  // Expected layout:
  // time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos
  int i1 = msg.indexOf(',');
  int i2 = msg.indexOf(',', i1+1);
  int i3 = msg.indexOf(',', i2+1);
  int i4 = msg.indexOf(',', i3+1);
  int i5 = msg.indexOf(',', i4+1);
  int i6 = msg.indexOf(',', i5+1);
  int i7 = msg.indexOf(',', i6+1);
  int i8 = msg.indexOf(',', i7+1);
  int i9 = msg.indexOf(',', i8+1);

  // Guard against malformed frames
  if (i1<0||i2<0||i3<0||i4<0||i5<0||i6<0||i7<0||i8<0) return;

  long   new_time_ms = msg.substring(0,   i1).toInt();
  int    new_lap     = msg.substring(i1+1,i2).toInt();
  int    new_speed   = msg.substring(i2+1,i3).toInt();
  int    new_gear    = msg.substring(i3+1,i4).toInt();
  int    new_thr     = msg.substring(i4+1,i5).toInt();
  int    new_brk     = msg.substring(i5+1,i6).toInt();
  int    new_drs     = msg.substring(i6+1,i7).toInt();
  int    new_rpm     = msg.substring(i7+1,i8).toInt();

  String new_flag;
  int    new_pos = 0;
  if (i9 >= 0) {
    new_flag = msg.substring(i8+1, i9);
    new_pos  = msg.substring(i9+1).toInt();
  } else {
    new_flag = msg.substring(i8+1);
    new_pos  = 0;
  }

  // -------- Lap transitions & delta computation --------
  // When we observe the lap number increment, we treat the previously tracked
  // lap duration as "completed" and update the lap delta string against the
  // immediately prior lap (if available).
  if (curLap < 0) {
    curLap = new_lap;
  } else if (new_lap > curLap) {
    long completedLapMs = lastTeleLapMs;
    if (completedLapMs >= 0) {
      lastLapMs = completedLapMs;
      if (prevLapMs >= 0) lastDeltaStr = fmtDeltaMsInt(lastLapMs - prevLapMs);
      else                lastDeltaStr = "   --.-";
      prevLapMs = lastLapMs;
    }
    curLap = new_lap;
  } else if (new_lap < curLap) {
    // Handle out-of-order or session restart; accept the new lap index
    curLap = new_lap;
  }

  // -------- Commit latest telemetry to globals --------
  time_ms = new_time_ms;
  lap     = new_lap;
  speed   = new_speed;
  gear    = new_gear;
  thr     = new_thr;      // Percent 0..100
  brk     = new_brk;      // Percent 0..100
  drs     = new_drs;      // Raw DRS code; treated as active by driveDRS_simple()
  rpm     = new_rpm;
  flag    = new_flag;
  pos     = new_pos;

  // -------- Drive physical outputs --------
  showGear(gear);         // Update the discrete 7-segment with the current gear
  driveDRS_simple(drs);   // DRS indicator LED
  setFlagLED(flag);       // R/Y/G flag LEDs

  // TM1637 speed display (range-limited to 0..9999)
  int spd = speed; if (spd < 0) spd = 0; if (spd > 9999) spd = 9999;
  spdDisplay.showNumberDec(spd, false);

  // 8×8 matrix bars: left block = brake, right block = throttle
  int hThr = pctToHeight(thr);
  int hBrk = pctToHeight(brk);
  fbClear();
  drawBar3Cols(0, hBrk);  // columns 0..2 = brake
  drawBar3Cols(5, hThr);  // columns 5..7 = throttle
  fbFlush();

  // -------- Smooth timing base for LCD extrapolation --------
  stream_active  = true;
  lastRxWallMs   = millis();
  lastTeleLapMs  = time_ms;
  lastTeleWallMs = lastRxWallMs;

  // Optional echo for logs/diagnostics (one line per frame)
  Serial.print("t_ms="); Serial.print(time_ms);
  Serial.print(" lap="); Serial.print(lap);
  Serial.print(" spd="); Serial.print(speed);
  Serial.print(" g=");   Serial.print(gear);
  Serial.print(" thr="); Serial.print(thr);
  Serial.print(" brk="); Serial.print(brk);
  Serial.print(" drs="); Serial.print(drs);
  Serial.print(" rpm="); Serial.print(rpm);
  Serial.print(" pos="); Serial.print(pos);
  Serial.print(" flag="); Serial.println(flag);
}

// =============== Flags (R/Y/G LED driver) ===============

void setFlagLED(const String& flg) {
  if (flg == "RED") {
    digitalWrite(FLAG_R_PIN, HIGH);
    digitalWrite(FLAG_G_PIN, LOW);
    digitalWrite(FLAG_Y_PIN, LOW);
  } else if (flg == "GREEN") {
    digitalWrite(FLAG_R_PIN, LOW);
    digitalWrite(FLAG_G_PIN, HIGH);
    digitalWrite(FLAG_Y_PIN, LOW);
  } else if (flg == "YELLOW") {
    digitalWrite(FLAG_R_PIN, LOW);
    digitalWrite(FLAG_G_PIN, LOW);
    digitalWrite(FLAG_Y_PIN, HIGH);
  }
}

// =============== Gear 7-segment (common-cathode) ===============

void showGear(int g) {
  if (g <= 0) { digital_dash(); return; }
  switch (g) {
    case 1: digital_1(); break; case 2: digital_2(); break; case 3: digital_3(); break;
    case 4: digital_4(); break; case 5: digital_5(); break; case 6: digital_6(); break;
    case 7: digital_7(); break; case 8: digital_8(); break; case 9: digital_9(); break;
    default: digital_9(); break;
  }
}

// =============== DRS Indicator (simple threshold) ===============

void driveDRS_simple(int v){
  // Treat these as "DRS active/open" signals
  if (v == 1 || v == 10 || v == 12 || v == 14 || v == 16) {
    digitalWrite(DRS_PIN, HIGH);
  } else {
    digitalWrite(DRS_PIN, LOW);
  }
}

// =============== MAX7219: bare-metal helpers ===============

/*
  MAX7219 note:
  -------------
  The matrix wiring/orientation means our logical (row, col) space must be
  rotated when sending to the device. fbFlush() handles this mapping so the
  rest of the code can think in a simple top-to-bottom, left-to-right grid.
*/

void maxSend(byte reg, byte data) {
  digitalWrite(MAX_CS, LOW);
  shiftOut(MAX_DIN, MAX_CLK, MSBFIRST, reg);
  shiftOut(MAX_DIN, MAX_CLK, MSBFIRST, data);
  digitalWrite(MAX_CS, HIGH);
}

void maxClear() {
  for (byte r = 1; r <= 8; r++) maxSend(r, 0x00);
}

void maxInit() {
  pinMode(MAX_DIN, OUTPUT);
  pinMode(MAX_CLK, OUTPUT);
  pinMode(MAX_CS,  OUTPUT);
  digitalWrite(MAX_CS, HIGH);

  maxSend(0x0F, 0x01); delay(50);  // display-test ON
  maxSend(0x0F, 0x00);             // display-test OFF
  maxSend(0x0C, 0x01);             // normal operation
  maxSend(0x0B, 0x07);             // scan limit: 8 rows
  maxSend(0x09, 0x00);             // decode mode OFF
  maxSend(0x0A, 0x08);             // medium intensity (0..15)
  maxClear();
}

// Framebuffer representing an 8×8 grid (row 0 = top)
byte fb[8];

void fbClear() { for (int i=0;i<8;i++) fb[i]=0; }

void fbSet(int row, int col, bool on){
  if (row<0||row>7||col<0||col>7) return;
  byte mask = (byte)1 << (7 - col);     // MSB corresponds to leftmost column
  if (on) fb[row] |= mask; else fb[row] &= ~mask;
}

bool fbGet(int row, int col){
  if (row<0||row>7||col<0||col>7) return false;
  byte mask = (byte)1 << (7 - col);
  return (fb[row] & mask) != 0;
}

void fbFlush(){
  // Rotate logical framebuffer 90° CCW into the device's row orientation.
  for (int rr = 0; rr < 8; ++rr) {   // rr = device row
    byte out = 0;
    for (int rc = 0; rc < 8; ++rc) {
      int orow = rc;
      int ocol = 7 - rr;
      if (fbGet(orow, ocol)) out |= (byte)1 << (7 - rc);
    }
    maxSend(rr + 1, out);
  }
}

// Draw a vertical bar using three adjacent columns
// colStart: leftmost column of the 3-wide bar (0..5)
void drawBar3Cols(int colStart, int height){
  if (colStart < 0 || colStart > 5) return;
  if (height < 0) height = 0; if (height > 8) height = 8;
  for (int row=0; row<8; ++row){
    bool on = (row >= 8 - height);   // bottom rows ON, top rows OFF
    for (int c=colStart; c<colStart+3; ++c) fbSet(row, c, on);
  }
}

// Map percent (0..100) to rows (0..8) with a small upward bias so that
// low non-zero values light at least one row around ~5%.
int pctToHeight(int pct){
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  return (pct == 0) ? 0 : ( (pct * 8 + 95) / 100 );
}

// =============== Utility helpers ===============

String fmtLapTimeMMSSmmm(long ms) {
  if (ms < 0) ms = 0;
  long minutes = ms / 60000L;
  long rem     = ms % 60000L;
  long seconds = rem / 1000L;
  long millis  = rem % 1000L;
  char buf[12];
  snprintf(buf, sizeof(buf), "%02ld:%02ld.%03ld", minutes, seconds, millis);
  return String(buf);
}

String fmtDeltaMsInt(long dms) {
  char sign = (dms >= 0) ? '+' : '-';
  unsigned long x = (dms >= 0) ? (unsigned long)dms : (unsigned long)(-dms);
  unsigned long sec = x / 1000UL;
  unsigned long mmm = x % 1000UL;
  char buf[12];
  snprintf(buf, sizeof(buf), "%c%lu.%03lu", sign, sec, mmm);
  return String(buf);
}

// Print exactly 16 characters to a given LCD row by composing a left and right label.
// Any overflow is truncated; unused space is padded with spaces.
void printRow16(uint8_t row, const String& left, const String& right) {
  char line[17]; for (int i=0;i<16;i++) line[i]=' '; line[16]='\0';
  int lLen = left.length(); if (lLen > 16) lLen = 16;
  for (int i=0; i<lLen; ++i) line[i] = left[i];
  int rLen = right.length(); if (rLen > 16) rLen = 16;
  int start = 16 - rLen; if (start < 0) start = 0;
  for (int i=0; i<rLen; ++i) line[start+i] = right[i];
  lcd.setCursor(0,row); lcd.print(line);
}

// Rate-limited LCD updater that extrapolates lap time while fresh data is flowing.
void updateLcdSmooth() {
  unsigned long now = millis();
  if (now - lastLcdTick < LCD_PERIOD_MS) return;
  lastLcdTick = now;

  bool fresh = stream_active && (now - lastRxWallMs <= STREAM_TIMEOUT_MS);

  long estLapMs = lastTeleLapMs;
  if (fresh) {
    estLapMs += (long)(now - lastTeleWallMs);
    if (estLapMs < 0) estLapMs = 0;
  }

  String timeStr  = fmtLapTimeMMSSmmm(estLapMs);
  String deltaStr = lastDeltaStr;
  String rpmStr   = String("RPM:") + String(rpm);
  String posStr   = (pos > 0) ? (String("P:") + String(pos)) : String("P:--");

  printRow16(0, timeStr, deltaStr);
  printRow16(1, rpmStr,  posStr);
}

// 7-segment digit glyphs (common-cathode)
void digital_0(void){
  digitalWrite(a,HIGH); digitalWrite(b,HIGH); digitalWrite(c,HIGH);
  digitalWrite(d,HIGH); digitalWrite(e,HIGH); digitalWrite(f,HIGH);
  digitalWrite(g,LOW);
}
void digital_1(void){
  digitalWrite(a,LOW);  digitalWrite(b,HIGH); digitalWrite(c,HIGH);
  digitalWrite(d,LOW);  digitalWrite(e,LOW);  digitalWrite(f,LOW);
  digitalWrite(g,LOW);
}
void digital_2(void){
  digitalWrite(a,HIGH); digitalWrite(b,HIGH); digitalWrite(c,LOW);
  digitalWrite(d,HIGH); digitalWrite(e,HIGH); digitalWrite(f,LOW);
  digitalWrite(g,HIGH);
}
void digital_3(void){
  digitalWrite(a,HIGH); digitalWrite(b,HIGH); digitalWrite(c,HIGH);
  digitalWrite(d,HIGH); digitalWrite(e,LOW);  digitalWrite(f,LOW);
  digitalWrite(g,HIGH);
}
void digital_4(void){
  digitalWrite(a,LOW);  digitalWrite(b,HIGH); digitalWrite(c,HIGH);
  digitalWrite(d,LOW);  digitalWrite(e,LOW);  digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
}
void digital_5(void){
  digitalWrite(a,HIGH); digitalWrite(b,LOW);  digitalWrite(c,HIGH);
  digitalWrite(d,HIGH); digitalWrite(e,LOW);  digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
}
void digital_6(void){
  digitalWrite(a,HIGH); digitalWrite(b,LOW);  digitalWrite(c,HIGH);
  digitalWrite(d,HIGH); digitalWrite(e,HIGH); digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
}
void digital_7(void){
  digitalWrite(a,HIGH); digitalWrite(b,HIGH); digitalWrite(c,HIGH);
  digitalWrite(d,LOW);  digitalWrite(e,LOW);  digitalWrite(f,LOW);
  digitalWrite(g,LOW);
}
void digital_8(void){
  digitalWrite(a,HIGH); digitalWrite(b,HIGH); digitalWrite(c,HIGH);
  digitalWrite(d,HIGH); digitalWrite(e,HIGH); digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
}
void digital_9(void){
  digitalWrite(a,HIGH); digitalWrite(b,HIGH); digitalWrite(c,HIGH);
  digitalWrite(d,HIGH); digitalWrite(e,LOW);  digitalWrite(f,HIGH);
  digitalWrite(g,HIGH);
}
void digital_dash(void){
  digitalWrite(a,LOW);  digitalWrite(b,LOW);  digitalWrite(c,LOW);
  digitalWrite(d,LOW);  digitalWrite(e,LOW);  digitalWrite(f,LOW);
  digitalWrite(g,HIGH);
}

