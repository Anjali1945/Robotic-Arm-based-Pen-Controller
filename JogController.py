# ================================================================
# Jog Controller
# Manually move the robotic arm along X, Y, Z axes using keyboard
# Used for calibration and positioning before a drawing session
# ================================================================

import serial
import time
import keyboard

# Open serial connection to Arduino running GRBL on COM9
ser = serial.Serial('COM9', 115200, timeout=1)
time.sleep(2)  # Wait for GRBL to initialize after connection

# Wake up GRBL and clear any startup messages from the buffer
ser.write(b"\r\n\r\n")
time.sleep(2)
ser.flushInput()

# --- Jog Settings ---
JOG_STEP = 5      # Distance to move per key press (mm)
FEED = 1000       # Movement speed (mm/min)

# Send a single command string to GRBL over serial
def send_cmd(cmd):
    ser.write((cmd + '\n').encode())
    print(">>", cmd)

# Send a GRBL jog command for the given axis and direction
# direction = 1 for positive, -1 for negative
# Uses G21 (mm units), G91 (relative positioning)
def jog(axis, direction):
    dist = JOG_STEP * direction
    cmd = f"$J=G21G91{axis}{dist}F{FEED}"
    send_cmd(cmd)

# Move arm back to origin (0, 0) in absolute mode
def go_home():
    send_cmd("G90")         # Switch to absolute positioning
    send_cmd("G0 X0 Y0")    # Rapid move to X=0, Y=0

print("🎮 JOG MODE STARTED")
print("Controls: WASD (XY), Q/E (Z), H=Home, ESC=Exit")

try:
    while True:

        # D key — move right along X axis
        if keyboard.is_pressed('d'):
            jog('X', 1)
            time.sleep(0.2)  # Debounce — prevents multiple triggers per press

        # A key — move left along X axis
        if keyboard.is_pressed('a'):
            jog('X', -1)
            time.sleep(0.2)

        # W key — move forward along Y axis
        if keyboard.is_pressed('w'):
            jog('Y', 1)
            time.sleep(0.2)

        # X key — move backward along Y axis
        if keyboard.is_pressed('x'):
            jog('Y', -1)
            time.sleep(0.2)

        # E key — move Z axis up (raise pen)
        if keyboard.is_pressed('e'):
            jog('Z', 1)
            time.sleep(0.2)

        # Q key — move Z axis down (lower pen)
        if keyboard.is_pressed('q'):
            jog('Z', -1)
            time.sleep(0.2)

        # H key — return arm to home position (X=0, Y=0)
        if keyboard.is_pressed('h'):
            print("🏠 Going Home")
            go_home()
            time.sleep(0.5)

        # ESC key — exit jog mode
        if keyboard.is_pressed('esc'):
            print("Exiting Jog Mode")
            break

except KeyboardInterrupt:
    pass  # Handle Ctrl+C gracefully without crashing

# Close the serial connection on exit
ser.close()
print("Disconnected")
