# ================================================================
# Robotic Arm Based Pen Controller
# Full pipeline: Camera Capture -> Skeletonization -> G-code -> Arduino
# ================================================================

# ---------------- PART 1: CAMERA CAPTURE ----------------

import cv2
import numpy as np

# Open the IVCam feed (index 1 = IVCam virtual webcam, CAP_DSHOW for Windows)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale to reduce complexity
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Median blur to reduce noise while preserving edges
    blur = cv2.medianBlur(gray, 5)

    # Adaptive thresholding — converts to binary image
    # Works better than fixed threshold under uneven lighting
    thresh = cv2.adaptiveThreshold(
        gray, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        15, 3
    )

    # Morphological opening — removes small noise dots from binary image
    kernel = np.ones((3,3), np.uint8)
    clean = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

    # Invert so strokes appear dark on white background for display
    final = cv2.bitwise_not(clean)

    cv2.imshow("Scanned Output", final)

    key = cv2.waitKey(1) & 0xFF

    # Press 'S' to snapshot the current frame and trigger the full pipeline
    if key == ord('s'):
        cv2.imwrite("rectangle.png", final)
        print("✅ Saved clean scanned image")

# ---------------- PART 2: SKELETON + GCODE ----------------

        from skimage.morphology import skeletonize

        # Reload the saved image in grayscale for processing
        img = cv2.imread("rectangle.png", 0)

        # --- Helper Functions ---

        # Check if two points are within a pixel threshold of each other
        def is_close(p1, p2, threshold=3):
            return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5 < threshold

        # Euclidean distance between two points
        def distance(p1, p2):
            return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

        # Merge nearby paths into one continuous path to reduce pen lifts
        def merge_paths(paths, threshold=5):
            merged = []
            while paths:
                current = paths.pop(0)
                i = 0
                while i < len(paths):
                    p = paths[i]
                    # If end of current path is close to start of another, join them
                    if distance(current[-1], p[0]) < threshold:
                        current.extend(p)
                        paths.pop(i)
                        i = 0
                    else:
                        i += 1
                merged.append(current)
            return merged

        # Threshold the saved image again for skeletonization
        _, thresh = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY_INV)
        binary = thresh > 0

        # Skeletonize — reduces thick strokes to single-pixel-wide paths
        skeleton = skeletonize(binary)
        skeleton = (skeleton * 255).astype(np.uint8)

        # Dilate and smooth to improve path continuity after skeletonization
        kernel = np.ones((3,3), np.uint8)
        skeleton = cv2.dilate(skeleton, kernel, iterations=1)
        skeleton = cv2.medianBlur(skeleton, 3)

        cv2.imwrite("skeleton2.png", skeleton)
        print("✅ Skeleton created")

        # Reload the skeleton image for path tracing
        img = cv2.imread("skeleton2.png", 0)

        # Get all non-zero (white) pixel coordinates
        points = cv2.findNonZero(img)
        points = [tuple(pt[0]) for pt in points]

        # Track which pixels have already been visited during path tracing
        visited = np.zeros_like(img)
        paths = []

        # 8-directional neighbors for tracing connected pixels
        neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]

        h, w = img.shape

        # Trace a connected path starting from pixel (x, y)
        # Follows neighbors one step at a time until no unvisited neighbor found
        def trace_path(x, y):
            path = [(x, y)]
            visited[y, x] = 1

            while True:
                found = False
                for dx, dy in neighbors:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if img[ny, nx] == 255 and not visited[ny, nx]:
                            x, y = nx, ny
                            path.append((x, y))
                            visited[ny, nx] = 1
                            found = True
                            break
                if not found:
                    break
            return path

        # Scan every pixel — start a new path from any unvisited white pixel
        for y in range(h):
            for x in range(w):
                if img[y, x] == 255 and not visited[y, x]:
                    paths.append(trace_path(x, y))

        print("✅ Paths extracted:", len(paths))

        # Save the starting position of the first path for homing later
        start_x, start_y = paths[0][0]

        # Remove duplicate path starts that are too close to each other
        filtered_paths = []
        for path in paths:
            if not any(is_close(path[0], p[0]) for p in filtered_paths):
                filtered_paths.append(path)

        paths = filtered_paths

        # Join nearby paths to minimize pen-up movements
        paths = merge_paths(paths)

        # Drop very short paths — likely noise, not worth drawing
        paths = [path for path in paths if len(path) > 20]

        # Remove duplicate points across all paths
        visited_points = set()
        clean_paths = []

        for path in paths:
            new_path = []
            for pt in path:
                if pt not in visited_points:
                    new_path.append(pt)
                    visited_points.add(pt)

            if len(new_path) > 5:
                clean_paths.append(new_path)

        paths = clean_paths

        # Redefine distance here for use in nearest-neighbor ordering
        def distance(p1, p2):
            return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

        # Order paths using nearest-neighbor — minimizes travel between strokes
        ordered_paths = []
        current = paths.pop(0)

        while paths:
            last_point = current[-1]
            next_path = min(paths, key=lambda p: distance(last_point, p[0]))
            paths.remove(next_path)
            ordered_paths.append(current)
            current = next_path

        ordered_paths.append(current)

        # Reduce number of points per path using Douglas-Peucker approximation
        # Keeps the shape accurate while generating fewer G-code lines
        def simplify_path(path, epsilon=2.0):
            pts = np.array(path, dtype=np.int32)
            approx = cv2.approxPolyDP(pts, epsilon, False)
            return [tuple(p[0]) for p in approx]

        # --- G-code Settings ---
        scale = 0.2       # Pixel to mm conversion factor
        FEED = 800        # Drawing speed (mm/min)
        RAPID = 1500      # Travel speed when pen is up (mm/min)

        Z_UP = 5          # Z height when pen is lifted (mm)
        Z_DOWN = 0        # Z height when pen is on paper (mm)

        with open("skeleton2.gcode", "w") as f:

            # Convert starting pixel position to mm
            start_x_mm = start_x * scale
            start_y_mm = (h - start_y) * scale  # Flip Y — image Y increases downward, CNC upward

            # Lift pen and move to start position before setting units/mode
            f.write(f"G0 Z{Z_UP}\n")
            f.write(f"G0 X{start_x_mm:.2f} Y{start_y_mm:.2f} F{RAPID}\n")

            f.write("G21\n")   # Set units to millimeters
            f.write("G90\n")   # Use absolute positioning

            f.write(f"G0 Z{Z_UP}\n")  # Ensure pen is up before first stroke

            for path in ordered_paths:
                if len(path) < 5:
                    continue  # Skip paths too short to be meaningful

                path = simplify_path(path, epsilon=2.0)

                # Move to the start of this path with pen up
                x0, y0 = path[0]
                x0 = x0 * scale
                y0 = (h - y0) * scale

                f.write(f"G0 Z{Z_UP}\n")                          # Lift pen
                f.write(f"G0 X{x0:.2f} Y{y0:.2f} F{RAPID}\n")   # Rapid move to path start
                f.write(f"G1 Z{Z_DOWN} F300\n")                   # Lower pen onto paper

                # Draw each point in the path
                for x, y in path:
                    x = x * scale
                    y = (h - y) * scale
                    f.write(f"G1 X{x:.2f} Y{y:.2f} F{FEED}\n")

                f.write(f"G0 Z{Z_UP}\n")  # Lift pen after finishing this stroke

            # Return to starting position after all paths are drawn
            f.write(f"G0 Z{Z_UP}\n")
            f.write(f"G0 X{start_x_mm:.2f} Y{start_y_mm:.2f} F{RAPID}\n")

        print("✅ G-code generated")

# ---------------- PART 3: UGS REPLICA ----------------
# Streams the generated G-code to Arduino over serial, with pause/resume/stop controls

        import serial
        import time
        import keyboard

        # Query GRBL for current machine status
        def get_status():
            ser.write(b'?')
            time.sleep(0.05)
            return ser.readline().decode(errors='ignore').strip()

        # Open serial connection to Arduino on COM9 at 115200 baud
        ser = serial.Serial('COM9', 115200, timeout=0.1)
        time.sleep(2)  # Wait for GRBL to initialize

        paused = False
        stopped = False

        # Send feed hold command to GRBL (soft pause)
        def pause_machine():
            global paused
            print("⏸ Paused")
            ser.write(b'!')
            paused = True

        # Send cycle start command to GRBL (resume)
        def resume_machine():
            global paused
            print("▶ Resumed")
            ser.write(b'~')
            paused = False
            ser.reset_input_buffer()

        # Send soft reset to GRBL (full stop, requires re-home)
        def stop_machine():
            global stopped
            print("⛔ Stopped")
            ser.write(b'\x18')
            stopped = True

        # Bind keyboard hotkeys to machine control functions
        keyboard.add_hotkey('p', pause_machine)
        keyboard.add_hotkey('r', resume_machine)
        keyboard.add_hotkey('s', stop_machine)

        # Wake up GRBL and clear any startup message in the buffer
        ser.write(b"\r\n\r\n")
        time.sleep(2)
        ser.flushInput()

        paused = False
        stopped = False

        print("Controls: P=Pause, R=Resume, S=Stop")

        # Read and send G-code line by line
        with open("skeleton2.gcode", "r") as f:
            for line in f:

                if stopped:
                    break  # Exit immediately if stop was triggered

                cmd = line.strip()
                if cmd == "":
                    continue  # Skip blank lines

                # Hold here if paused — wait until resumed
                while paused:
                    time.sleep(0.1)

                print("Sending:", cmd)
                ser.reset_input_buffer()
                ser.write((cmd + '\n').encode())

                # Small extra delay for drawing moves to avoid buffer overflow
                if cmd.startswith("G1"):
                    time.sleep(0.01)

                time.sleep(0.02)
                start_time = time.time()

                # Wait for GRBL to acknowledge the command with "ok"
                while True:
                    response = ser.readline().decode(errors='ignore').strip()

                    if response:
                        print("GRBL:", response)

                        if "ok" in response.lower():
                            break  # Command accepted, move to next line

                    if stopped:
                        break

                    # Timeout after 5 seconds — don't hang indefinitely
                    if time.time() - start_time > 5:
                        print("⚠️ No response, continuing...")
                        break

        ser.close()
        print("Program ended")

    # Press ESC to exit the live camera feed
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
