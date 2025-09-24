import serial
import re
import time

# Serial connection to the Arduino (adjust as needed)
ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)

# Data log to store position readings
position_log = []

# Regex patterns for x and y
x_pattern = re.compile(r'x:\s*(-?[\d.]+)', re.IGNORECASE)
y_pattern = re.compile(r'y:\s*(-?[\d.]+)', re.IGNORECASE)

print("Reading Arduino position data...")

try:
    buffer = ""
    while True:
        try:
            # Read a line and append it to the buffer
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            buffer += line + '\n'

            # Check if buffer contains a complete block
            if 'x:' in buffer and 'y:' in buffer:
                x_match = x_pattern.search(buffer)
                y_match = y_pattern.search(buffer)

                if x_match and y_match:
                    x_val = float(x_match.group(1))
                    y_val = float(y_match.group(1))

                    reading = {'x': x_val, 'y': y_val}
                    position_log.append(reading)
                    print(reading)

                buffer = ""  # Clear buffer for next block

        except Exception as e:
            print(f"Warning: failed to process block — {e}")
            buffer = ""  # Reset buffer on error

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Interrupted. Closing serial port.")
    ser.close()
