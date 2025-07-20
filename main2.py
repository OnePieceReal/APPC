#!/usr/bin/env python3
"""
Integrated Robot Control with Priority:
  1. Obstacle Avoidance (always active)
  2. Drop-off Zone Detection & Ball Release
  3. Ping-Pong Ball Collection
  4. Continuous Autofocus

Behavior:
   The LiDAR thread continuously scans and updates global variables (avoidance_active and avoidance_maneuver)
    based on obstacles detected in a defined front sector.
   The detection thread uses a camera and Jetson Inference to detect ping-pong balls ("orange_pp")
    and HSV masking to detect blue drop-off zones, updating global ball_target and drop_off_detected.
   The autofocus thread continuously runs an external autofocus script ("Focuser.py") to adjust focus.
   The main control loop always first checks for obstacles:
       - If an obstacle is detected, it executes the avoidance maneuver.
       - If no obstacle is present, it checks for drop-off zones:
             * If a drop-off zone is detected, it stops the robot and releases the ball.
       - If no drop-off zone is detected, it checks for balls:
             * If a ball is detected, it runs the collection algorithm.
             * Once the ball is collected or lost, control immediately reverts to obstacle avoidance.
   On termination (via KeyboardInterrupt), the LiDAR thread stops the motor and disconnects the sensor.
  
Make sure that LiDAR and Arduino use separate serial ports.
"""

import time
import threading
import subprocess
import serial
import math
import numpy as np
import RPi.GPIO as GPIO
import cv2
from rplidar import RPLidar
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

# ----------------------- Global Definitions -----------------------

# GPIO pin definitions (adjust as needed)
ENA, IN1, IN2 = 33, 37, 35
ENB, IN3, IN4 = 32, 40, 38

running = True  # Global run flag

# Global obstacle avoidance variables (updated only by the LiDAR thread)
avoidance_active = False       # True if an obstacle is detected
avoidance_maneuver = None      # "left", "right", or "pivot"
avoidance_lock = threading.Lock()  # Protects the above variables

# Global ball target (updated by the detection thread)
ball_target = None
ball_target_lock = threading.Lock()

# Global drop-off zone detection (updated by the detection thread)
drop_off_detected = False
drop_off_lock = threading.Lock()

# Serial communication settings (ensure LiDAR and Arduino are on separate ports)
arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
LIDAR_PORT = "/dev/ttyUSB0"

# ----------------------- HSV Masking Functions -----------------------

def hex_to_hsv(hex_color):
    """Convert hex color to HSV"""
    hex_color = hex_color.lstrip('#')
    rgb = tuple(int(hex_color[i:i + 2], 16) for i in (0, 2, 4))
    bgr = np.uint8([[rgb[::-1]]])  # Convert RGB to BGR
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0][0]
    return hsv

def detect_drop_off_zone(frame):
    """Detect blue drop-off zone using HSV masking"""
    # Define the HSV bounds from #00008A to #0000FF
    hsv_min = hex_to_hsv("#00008A")
    hsv_max = hex_to_hsv("#0000FF")

    # Ensure proper ordering
    lower_bound = np.minimum(hsv_min, hsv_max)
    upper_bound = np.maximum(hsv_min, hsv_max)

    # Add margin to make detection more flexible
    margin = np.array([10, 50, 50])
    lower_bound = np.maximum(lower_bound - margin, [0, 0, 0])
    upper_bound = np.minimum(upper_bound + margin, [179, 255, 255])

    # Convert frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create binary mask for blue
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Optional: clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 10000:  # Only consider large zones
            detected = True
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"Drop-off Zone: {int(area)}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if detected:
        cv2.putText(frame, "DROP-OFF DETECTED", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    return detected

# ----------------------- Motor Control Functions -----------------------

def forward():
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(ENB, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    print("Moving forward")

def left():
    GPIO.output(ENA, GPIO.LOW)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(ENB, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    print("Turning left")

def right():
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(ENB, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    print("Turning right")

def leftPivot():
    GPIO.output(ENA, GPIO.HIGH)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(ENB, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    print("Left pivot")

def kill():
    GPIO.output(ENA, GPIO.LOW)
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(ENB, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    print("Motors stopped")

def sendUART(ardcom):
    ardcomChar = str(ardcom)
    print(f"Sending UART command: {ardcomChar}")
    arduino.write(ardcomChar.encode())
    arduino.flush()

# ----------------------- GPIO Setup -----------------------

def setup_gpio():
    GPIO.setmode(GPIO.BOARD)
    for pin in [ENA, IN1, IN2, ENB, IN3, IN4]:
        GPIO.setup(pin, GPIO.OUT)

# ----------------------- LiDAR Obstacle Avoidance Thread -----------------------

def lidar_thread():
    global running, avoidance_active, avoidance_maneuver
    print("LiDAR thread starting...")
    # Set angle thresholds and distance threshold (adjust as needed)
    FRONT_START = 55    # degrees
    FRONT_END   = 125   # degrees
    DIST_THRESHOLD = 475  # WAS 450 mm

    try:
        lidar = RPLidar(LIDAR_PORT)
        print("LiDAR initialized on port:", LIDAR_PORT)
        # Uncomment if your LiDAR requires explicit motor start:
        # lidar.start_motor()
        # time.sleep(3)
    except Exception as e:
        print("Failed to initialize LiDAR:", e)
        return

    try:
        for scan in lidar.iter_scans():
            if not running:
                break

            front_dists = []
            left_dists = []
            right_dists = []
            for (_, angle, distance) in scan:
                if distance < DIST_THRESHOLD:
                    if FRONT_START <= angle <= FRONT_END:
                        front_dists.append(distance)
                    elif angle < FRONT_START:
                        left_dists.append(distance)
                    elif angle > FRONT_END:
                        right_dists.append(distance)

            with avoidance_lock:
                if len(front_dists) == 0:
                    avoidance_active = False
                    avoidance_maneuver = None
                else:
                    avoidance_active = True
                    avg_left = sum(left_dists) / len(left_dists) if left_dists else float('inf')
                    avg_right = sum(right_dists) / len(right_dists) if right_dists else float('inf')
                    if avg_left > avg_right:
                        avoidance_maneuver = "left"
                    elif avg_right > avg_left:
                        avoidance_maneuver = "right"
                    else:
                        avoidance_maneuver = "pivot"

            print("LiDAR scan points:", len(scan))
            print("Front distances:", front_dists)
            print("Left distances:", left_dists)
            print("Right distances:", right_dists)
            with avoidance_lock:
                print("Avoidance active:", avoidance_active, "Maneuver:", avoidance_maneuver)
            time.sleep(0.1)
    except Exception as e:
        print("LiDAR thread exception:", e)
    finally:
        try:
            print("Stopping LiDAR motor...")
            lidar.stop_motor()
        except Exception as e:
            print("Error stopping LiDAR motor:", e)
        try:
            print("Disconnecting LiDAR...")
            lidar.disconnect()
        except Exception as e:
            print("Error disconnecting LiDAR:", e)
        print("LiDAR thread terminated.")

# ----------------------- Detection Thread -----------------------

def detection_thread():
    global running, ball_target, drop_off_detected
    net = detectNet(argv=[
        "--model=/home/capstone/Desktop/maincapstone/pingPongBallCollector/608Final/ssd-mobilenet.onnx",
        "--labels=/home/capstone/Desktop/maincapstone/pingPongBallCollector/608Final/labels.txt",
        "--input-blob=input_0",
        "--output-cvg=scores",
        "--output-bbox=boxes",
        "--threshold=0.35"
    ], threshold=0.40)
    camera = videoSource("csi://0")
    display = videoOutput("display://0")
    
    while running:
        img = camera.Capture()
        if img is None:
            time.sleep(0.05)
            continue

        # Detect drop-off zone using HSV masking
        drop_off_found = detect_drop_off_zone(img)
        with drop_off_lock:
            drop_off_detected = drop_off_found

        # Ball detection using Jetson Inference
        detections = net.Detect(img)
        maxObj = None
        for obj in detections:
            if obj.ClassID == 1:  # "orange_pp"
                if maxObj is None or obj.Area > maxObj.Area:
                    maxObj = obj

        with ball_target_lock:
            if maxObj is not None:
                ball_target = {
                    "center": maxObj.Center,
                    "top": maxObj.Top,
                    "bottom": maxObj.Bottom,
                    "left": maxObj.Left,
                    "right": maxObj.Right
                }
            else:
                ball_target = None

        # Render the frame with both detections
        display.Render(img)
        status_text = f"Detection FPS: {net.GetNetworkFPS():.0f}"
        if drop_off_found:
            status_text += " | Drop-off Zone Detected!"
        display.SetStatus(status_text)
        time.sleep(0.2)
    camera.Close()
    display.Close()

# ----------------------- Autofocus Thread -----------------------

def autofocus_thread():
    # Continuously run an external autofocus script (Focuser.py)
    # Adjust the sleep interval as needed
    while running:
        print("Autofocus thread: starting autofocus routine...")
        try:
            result = subprocess.run("python3 Focuser.py", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout = result.stdout.decode().strip()
            stderr = result.stderr.decode().strip()
            if stdout:
                print("Autofocus output:", stdout)
            if stderr:
                print("Autofocus error:", stderr)
        except Exception as e:
            print("Autofocus thread exception:", e)
        # Wait a bit before re-running autofocus
        time.sleep(1)

# ----------------------- Main Control Loop (State Machine) -----------------------

def main_control_loop():
    global running, ball_target
    print("Starting main control loop...")
    while running:
        # Priority 1: Obstacle avoidance (always active)
        with avoidance_lock:
            obstacle = avoidance_active
            maneuver = avoidance_maneuver

        if obstacle:
            print("State: Obstacle Avoidance")
            if maneuver == "left":
                left()
            elif maneuver == "right":
                right()
            else:
                leftPivot()
        else:
            # Priority 2: Check for drop-off zone detection
            with drop_off_lock:
                drop_off_found = drop_off_detected

            if drop_off_found:
                print("State: Drop-off Zone Detected - Releasing Ball")
                kill()  # Stop the robot
                sendUART(2)  # Open back door to release ball
                time.sleep(3)  # Wait for door to open and ball to fall
                sendUART(0)  # Close the door
                print("Ball release sequence completed")
                # Clear the drop-off detection to continue normal operation
                with drop_off_lock:
                    drop_off_detected = False
                time.sleep(1)  # Brief pause before resuming
            else:
                # Priority 3: Object detection & collection
                with ball_target_lock:
                    current_ball = ball_target.copy() if ball_target is not None else None

                if current_ball is not None:
                    print("State: Object Detected")
                    cx = current_ball["center"][0]
                    top_val = current_ball["top"]
                    print(f"Pursuing ball at X={cx}, Top={top_val}")
                    if top_val > 400:
                        kill()
                        if cx > 900:
                            right()
                        elif cx < 420:
                            left()
                        else:
                            print("Ball centered  entering collection phase")
                            collection_duration = 3  # seconds, adjust as needed
                            sendUART(1)  # Start collection command
                            start_time = time.time()
                            while time.time() - start_time < collection_duration:
                                forward()  # Keep driving forward during collection phase
                                time.sleep(0.1)  # Small delay to let the command take effect
                            sendUART(3)  # Return to normal mode after collection phase
                            with ball_target_lock:
                                ball_target = None
                    else:
                        if cx > 870:
                            right()
                        elif cx < 430:
                            left()
                        else:
                            forward()
                else:
                    print("State: Roaming")
                    forward()
        time.sleep(0.1)

# ----------------------- Main -----------------------

def main():
    global running
    print("Initializing resources...")
    setup_gpio()

    # Start the LiDAR, detection, and autofocus threads.
    lidar_thread_obj = threading.Thread(target=lidar_thread)
    detection_thread_obj = threading.Thread(target=detection_thread)
    autofocus_thread_obj = threading.Thread(target=autofocus_thread)
    lidar_thread_obj.start()
    detection_thread_obj.start()
    autofocus_thread_obj.start()

    # Allow sensors to warm up.
    time.sleep(3)
    print("Initialization complete. Starting motion...")

    try:
        main_control_loop()
    except KeyboardInterrupt:
        print("KeyboardInterrupt received. Stopping...")
        running = False

    lidar_thread_obj.join()
    detection_thread_obj.join()
    autofocus_thread_obj.join()
    kill()
    GPIO.cleanup()
    sendUART(2)
    time.sleep(3)
    sendUART(0)
    print("Program terminated.")

if __name__ == "__main__":
    main()
