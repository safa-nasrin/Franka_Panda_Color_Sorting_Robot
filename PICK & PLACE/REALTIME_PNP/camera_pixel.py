"""
// Calibration Tool: Click 4 reference points on the table to map camera pixels.
// Captures precise coordinates for coordinate transformation and vision alignment.
"""

import cv2
import numpy as np

# This stores the points you click
points = []

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append([x, y])
        print(f"Clicked Point {len(points)}: Pixel X={x}, Y={y}")
        
        # Draw a red dot where you clicked
        cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Calibration", frame)

# Open Camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

print("Please click on your 4 reference points on the screen...")

while len(points) < 4:
    ret, frame = cap.read()
    if not ret: break
    
    # Apply your exact rotation/warp so it matches your main code
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    h, w = frame.shape[:2]
    frame = cv2.warpAffine(frame, cv2.getRotationMatrix2D((w//2, h//2), 6.5, 1.0), (w, h))
    
    # Show video and wait for clicks
    cv2.imshow("Calibration", frame)
    cv2.setMouseCallback("Calibration", click_event)
    
    cv2.waitKey(1)

print("\n✅ DONE! Copy this into your main code for 'cam_pixels':")
print(f"cam_pixels = np.array({points}, dtype=float)")

cap.release()
cv2.destroyAllWindows()
