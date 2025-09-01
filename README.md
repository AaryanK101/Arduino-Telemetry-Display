**Telemetry Display: Arduino + FastF1/OpenF1**

A hardware dashboard that replays Formula 1 telemetry on real devices. Speed, gear, brake, throttle, flags, DRS, lap timer, delta, RPM, and live position are rendered on inexpensive modules driven by an Arduino. Python fetches and paces data, then streams a compact CSV protocol over serial.

**Features**

Speed on TM1637 4-digit module

Gear on a dedicated 7-segment

Brake and throttle as 3-column bars on an 8×8 MAX7219 matrix

Track flags on R/Y/G LEDs

DRS indicator LED with code normalization

LCD status line: lap time, delta, RPM, race position

Historical replay using FastF1 with real-time pacing

Live position merge via OpenF1, with graceful fallback to per-lap position


**System architecture**

1. **Data layer**

  FastF1 loads a session’s laps and per-sample car data: Time, Speed, nGear, Throttle, Brake, DRS, RPM
  
  Track status messages are converted to GREEN / YELLOW / RED
  
  OpenF1 position samples are joined on session time. If no samples, the system falls back to each lap’s end position.

2. **Transport**

  Python paces playback to wall-clock time and sends CSV lines over serial at 115200 baud
  
  Simple handshake: Python sends PING, Arduino replies PONG-A
  
  Header: HDR,time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos
  
  Example line: 35124,8,267,6,72,00,12,12450,GREEN,5

3. **Render layer (Arduino)**

  Parses each CSV field by index
  
  Drives: TM1637 speed, 7-segment gear, MAX7219 bars, LEDs, and I²C LCD
  
  Smooth UI: a 50 ms LCD scheduler estimates in-between time so the clock ticks even between serial frames
  
  DRS codes treated as active for {1, 10, 12, 14, 16}


**Hardware**

MCU: Arduino Uno

  LCD: 16×2 hd44780 with I²C backpack (PCF8574) on SDA/SCL
  
  Speed: TM1637 4-digit 7-segment
  
  Gear: single 7-segment, pins a..g wired individually
  
  Bars: 8×8 LED matrix via MAX7219
  
  LEDs: Red, Yellow, Green for flags, and one for DRS
  

**Pin map**

  **LCD (I²C)**
SDA = A4
SCL = A5

  **TM1637 speed**
CLK = D11
DIO = D12

  **MAX7219 matrix**
DIN = A0
CS = D2
CLK = D3

  **Gear 7-segment**
a = D9
b = D8
c = D4
d = D5
e = D6
f = D10
g = D13

  **LEDs**
DRS = D7
Flag-R = A1
Flag-Y = A2
Flag-G = A3


**Software**

  **1. Python Stack**
  
    fastf1 for telemetry and track status
    
    requests for OpenF1 REST calls
    
    pandas for joining and time alignment
    
    pyserial for serial IO
  
  **2. Arduino stack**
  
    Wire and hd44780_I2Cexp for LCD
    
    TM1637Display for speed
    
    Bare-metal shiftOut for MAX7219
    
    Direct pin control for the gear 7-segment and LEDs

**3. CSV protocol**

Header:
HDR,time_ms,lap,speed,gear,thr,brk,drs,rpm,flag,pos


**Fields**

time_ms: milliseconds since lap start

lap: current lap number

speed: km/h

gear: 0..9

thr: throttle percent 0–100

brk: brake percent 0–100

drs: raw DRS code normalized to integers {0,1,10,12,14,16}

rpm: engine RPM

flag: GREEN | YELLOW | RED

pos: race position integer, 0 if unknown
