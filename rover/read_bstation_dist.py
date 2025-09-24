'''
    This script is designed to read the serial data coming in from
    the DWM1001-DEV board on the rover's Raspberry Pi. We use minicom
    to communicate with the DEV board via a usb connection. The data 
    being send from the DEV board is the distances of the rover to the 
    base stations, and these are used to produce an accurate pose estimate.

    The data is printed out in the form:
    UWB-ID[x,y,z]=distance UWB-ID2[x2,y2,z2]=distance2 etc...

    Here's an example print-out:
    0F9C[0.00,0.00,0.00]=0.20 0D10[0.00,0.00,0.00]=0.41 
'''

import serial
import re
import time

# ==== SET TARGET ANCHORS HERE ====
TARGET_ANCHORS = ["0F9C", "0D10"]  # Use uppercase hex strings
# ======================================

# Serial port setup
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Log to store parsed anchor data
data_log = []

# Regex pattern for a line like: 0F9C[0.00,0.00,0.00]=0.20
pattern = re.compile(r'([0-9A-F]{4})\[(.*?)\]=([\d.]+)')

print("Reading serial data...")

try:
    while True:
        try:
            raw = ser.readline().decode('utf-8', errors='ignore').strip()
            if not raw:
                continue

            # Handle multiple lines in one read
            lines = raw.splitlines()

            for line in lines:
                matches = pattern.findall(line)
                if matches:
                    entry = []
                    for match in matches:
                        try:
                            anchor_id = match[0].upper()
                            if anchor_id not in TARGET_ANCHORS:
                                continue  # Skip irrelevant anchors

                            position = list(map(float, match[1].split(',')))
                            distance = float(match[2])
                            entry.append({
                                'anchor_id': anchor_id,
                                'position': position,
                                'distance': distance
                            })
                        except Exception as e:
                            print(f"Warning: error parsing match {match} — {e}")
                            continue
                    if entry:
                        data_log.append(entry)
                        print(entry)

        except Exception as e:
            print(f"Warning: failed to process line: {raw} — {e}")
            continue

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Interrupted by user. Closing serial port.")
    ser.close()
