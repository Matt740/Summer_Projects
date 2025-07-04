from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize camera
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
picam2.start()

cv2.waitKey(1000)  # Let camera warm up

# Trackbar callback
def nothing(x):
    pass

# Create a window with HSV tuning sliders
cv2.namedWindow("Mask Tuner")
cv2.createTrackbar("H Low", "Mask Tuner", 10, 179, nothing)
cv2.createTrackbar("H High", "Mask Tuner", 35, 179, nothing)
cv2.createTrackbar("S Low", "Mask Tuner", 0, 255, nothing)
cv2.createTrackbar("S High", "Mask Tuner", 255, 255, nothing)
cv2.createTrackbar("V Low", "Mask Tuner", 160, 255, nothing)
cv2.createTrackbar("V High", "Mask Tuner", 255, 255, nothing)

while True:
    # Capture frame and convert to RGB
    frame = picam2.capture_array()

    # Convert to HSV for color filtering
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # Get current HSV values from trackbars
    h_low = cv2.getTrackbarPos("H Low", "Mask Tuner")
    h_high = cv2.getTrackbarPos("H High", "Mask Tuner")
    s_low = cv2.getTrackbarPos("S Low", "Mask Tuner")
    s_high = cv2.getTrackbarPos("S High", "Mask Tuner")
    v_low = cv2.getTrackbarPos("V Low", "Mask Tuner")
    v_high = cv2.getTrackbarPos("V High", "Mask Tuner")

    # Define lower and upper HSV range
    lower_orange = np.array([h_low, s_low, v_low])
    upper_orange = np.array([h_high, s_high, v_high])

    # Create binary mask where orange colors are white
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    blurred = cv2.GaussianBlur(mask, (9, 9), 2)

    # Use Hough Transform to detect circles
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=50,
        param1=100,
        param2=30,
        minRadius=10,
        maxRadius=100
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            # Draw circle and center
            cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame, (x, y), 3, (0, 0, 255), -1)
            print(f"Ball position: x={x}, y={y}, radius={r}")
    
    # Show outputs
    DisplayImage = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    cv2.imshow("Ball Tracking", DisplayImage)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup
cv2.destroyAllWindows()
picam2.stop()
