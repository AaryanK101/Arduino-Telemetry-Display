import time, serial

PORT = "COM6"     # your Arduino Uno is on COM6
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.5)
time.sleep(2.0)   # wait for Arduino reset after port open

ser.write(b"PING\n")
print("Sent: PING")
print("Got:", ser.readline().decode(errors="ignore").strip())

for i in range(3):
    line = f"{i*1000},1,{100+i},3,90,0,{12000+i},GREEN\n"
    ser.write(line.encode())
    echo = ser.readline().decode(errors="ignore").strip()
    print("Echo:", echo)
    time.sleep(0.2)

ser.close()