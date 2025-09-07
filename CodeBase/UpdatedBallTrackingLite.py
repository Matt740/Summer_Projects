from picamera2 import Picamera2
import cv2
import numpy as np
import time

class BallTrackerLite:
    def __init__(
        self,
        resolution=(640, 480),
        hsv_low=(9, 60, 160),     # defaults tuned for orange-ish
        hsv_high=(38, 255, 255),
        hough_dp=1.2,
        hough_minDist=50,
        hough_param1=100,
        hough_param2=30,
        minRadius=10,
        maxRadius=100,
        blur_ksize=9,
        blur_sigma=2,
    ):
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": resolution}))
        self.W, self.H = resolution
        self.cx, self.cy = self.W // 2, self.H // 2

        self.lower = np.array(hsv_low, dtype=np.uint8)
        self.upper = np.array(hsv_high, dtype=np.uint8)

        self.hough_dp = hough_dp
        self.hough_minDist = hough_minDist
        self.hough_param1 = hough_param1
        self.hough_param2 = hough_param2
        self.minRadius = minRadius
        self.maxRadius = maxRadius

        self.blur_ksize = (blur_ksize, blur_ksize)
        self.blur_sigma = blur_sigma

        self._started = False

    def set_hsv(self, low, high):
        """Update HSV thresholds at runtime."""
        self.lower = np.array(low, dtype=np.uint8)
        self.upper = np.array(high, dtype=np.uint8)

    def start(self):
        if not self._started:
            self.picam2.start()
            # brief warmup for exposure/gain
            time.sleep(0.3)
            self._started = True

    def stop(self):
        if self._started:
            self.picam2.stop()
            self._started = False

    def close(self):
        self.stop()

    def read(self):
        """
        Returns dict with pixel + normalized coords, or None if no ball:
        {
          "x": px, "y": px, "r": px,
          "u": [-1..1], "v": [-1..1],
          "timestamp": float
        }
        """
        frame_rgb = self.picam2.capture_array()
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.GaussianBlur(mask, self.blur_ksize, self.blur_sigma)

        circles = cv2.HoughCircles(
            mask,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_minDist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.minRadius,
            maxRadius=self.maxRadius
        )

        if circles is None:
            return None

        circles = np.round(circles[0, :]).astype("int")
        x, y, r = max(circles, key=lambda c: c[2])  # pick the largest circle

        u = (x - self.cx) / (self.W / 2.0)  # normalized offsets for PID
        v = (y - self.cy) / (self.W / 2.0) # normalizing to the width as well to maintain spatial consistency

        return {
            "x": int(x),
            "y": int(y),
            "r": int(r),
            "u": float(u),
            "v": float(v),
            "timestamp": time.time()
        }

# # -------- Minimal usage --------
# if __name__ == "__main__":
#     tracker = BallTrackerLite()
#     tracker.start()
#     try:
#         while True:
#             pos = tracker.read()
#             if pos:
#                 # Feed pos["u"], pos["v"] into your PID as error signals
#                 print(pos)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         tracker.close()
