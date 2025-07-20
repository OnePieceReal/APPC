import cv2
import numpy as np

def hex_to_hsv(hex_color):
    """Convert hex color to HSV"""
    hex_color = hex_color.lstrip('#')
    rgb = tuple(int(hex_color[i:i + 2], 16) for i in (0, 2, 4))
    bgr = np.uint8([[rgb[::-1]]])  # Convert RGB to BGR
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0][0]
    return hsv

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

print(f"Detecting HSV range: {lower_bound} to {upper_bound}")

# Start camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    frame = cv2.flip(frame, 1)
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
            cv2.putText(frame, f"Area: {int(area)}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    if detected:
        print("Blue drop-off zone detected!")
        cv2.putText(frame, "DETECTED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    cv2.imshow("Blue Drop-Off Zone Detection", frame)
    # Optional debug view:
    # cv2.imshow("Mask", mask)

    key = cv2.waitKey(1)
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
