import serial
import re
import time
import numpy as np

# --- Data Classes ---

class BaseStation:
    def __init__(self, anchor_id, x, y):
        self.anchor_id = anchor_id
        self.x = x
        self.y = y
        self.dist = -1  # Unknown initially

# --- Bilateration Function ---

def trilaterate_two_anchors(base1, base2):
    D = np.hypot(base2.x - base1.x, base2.y - base1.y)
    if (base1.dist == -1) or (base2.dist == -1):
        raise ValueError("No distance has been read yet, cannot trilaterate.")
    print(f"D={D}, base1.dist:{base1.dist}, base2.dist:{base2.dist}")
    if (D > base1.dist + base2.dist) or (D < abs(base1.dist - base2.dist)) or (D == 0):
        raise ValueError("No intersection or infinite solutions (bad input).")
    a = (base1.dist**2 - base2.dist**2 + D**2) / (2 * D)
    h = np.sqrt(base1.dist**2 - a**2)
    x3 = base1.x + a * (base2.x - base1.x) / D
    y3 = base1.y + a * (base2.y - base1.y) / D
    rx = -(base2.y - base1.y) * (h / D)
    ry =  (base2.x - base1.x) * (h / D)
    sol1 = (x3 + rx, y3 + ry)
    sol2 = (x3 - rx, y3 - ry)
    return sol1, sol2

# --- Anchor Setup ---

base1 = BaseStation("0F9C", 0.0, 0.0)
base2 = BaseStation("0D10", 1.0, 0.0)

# --- Serial Setup ---

anchor_ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
odom_ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)

anchor_pattern = re.compile(r'([0-9A-F]{4})\[(.*?)\]=([\d.]+)')
x_pattern = re.compile(r'x:\s*(-?[\d.]+)')
y_pattern = re.compile(r'y:\s*(-?[\d.]+)')

print("Running trilateration with live data...")

try:
    while True:
        # --- Get anchor distances ---
        try:
            line = anchor_ser.readline().decode('utf-8', errors='ignore').strip()
            matches = anchor_pattern.findall(line)
            for m in matches:
                anchor_id = m[0].upper()
                dist = float(m[2])
                if anchor_id == base1.anchor_id:
                    base1.dist = dist
                elif anchor_id == base2.anchor_id:
                    base2.dist = dist
        except Exception:
            pass

        # --- Get odometry position ---
        try:
            block = ""
            for _ in range(3):
                line = odom_ser.readline().decode('utf-8', errors='ignore').strip()
                block += line + "\n"
            x_match = x_pattern.search(block)
            y_match = y_pattern.search(block)
            if x_match and y_match:
                odom_x = float(x_match.group(1))
                odom_y = float(y_match.group(1))
                odom_pos = np.array([odom_x, odom_y])
        except Exception:
            continue

        # --- Perform trilateration ---
        if base1.dist != -1 and base2.dist != -1:
            try:
                pos1, pos2 = trilaterate_two_anchors(base1, base2)
                p1 = np.array(pos1)
                p2 = np.array(pos2)

                if np.linalg.norm(p1 - odom_pos) < np.linalg.norm(p2 - odom_pos):
                    est = p1
                else:
                    est = p2

                print(f"Estimated position: {est}")
            except Exception as e:
                print(f"[!] Trilateration error: {e}")
        else:
            print("[!] Waiting for both anchor distances...")

        time.sleep(0.2)

except KeyboardInterrupt:
    print("Exiting.")
    anchor_ser.close()
    odom_ser.close()
