import serial

PORT = "/dev/ttyUSB0"   # change if needed
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Connected to ESP32")

while True:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print("ESP32:", line)
