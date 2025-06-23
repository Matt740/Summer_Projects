from picamera2 import Picamera2
import cv2
import time

# Initialize camera
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")

picam2.start()
time.sleep(1)  # Let the camera warm up

print("? Camera started. Press 'q' to quit.")

while True:
    frame = picam2.capture_array()
    cv2.imshow("Camera", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
