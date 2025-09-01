# check_com5.py
import time, sys
try:
    import serial, serial.tools.list_ports
except ImportError:
    print("pip install pyserial"); sys.exit(1)

print("[Ports detected]")
for p in serial.tools.list_ports.comports():
    print(f" {p.device:>6}  {p.description}")

PORT = "COM6"  # change if Device Manager shows a different COM
BAUD = 115200

try:
    s = serial.Serial(PORT, BAUD, timeout=0.5)
except Exception as e:
    print(f"[OPEN ERROR] {e}")
    sys.exit(1)

time.sleep(2.0)                 # let the board reset
s.reset_input_buffer()
s.write(b"PING\n")
echo = s.readline().decode(errors="ignore").strip()
print(f"[READ] {echo if echo else '(no reply)'}")
s.close()
