from picamera2 import Picamera2
import cv2
import numpy as np
import time

class BallTracker:
    def __init__(
        self,
        resolution=(640, 480),
        show_tuner=False,
        hsv_init=(9, 38, 59, 255, 160, 255),  # (Hlow, Hhigh, Slow, Shigh, Vlow, Vhigh)
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

        self.show_tuner = show_tuner
        self._hsv = list(hsv_init)  # mutable

        # Hough params
        self.hough_dp = hough_dp
        self.hough_minDist = hough_minDist
        self.hough_param1 = hough_param1
        self.hough_param2 = hough_param2
        self.minRadius = minRadius
        self.maxRadius = maxRadius

        # Blur params
        self.blur_ksize = (blur_ksize, blur_ksize)
        self.blur_sigma = blur_sigma

        # UI
        if self.show_tuner:
            cv2.namedWindow("Mask Tuner", cv2.WINDOW_NORMAL)
            cv2.createTrackbar("H Low",  "Mask Tuner", self._hsv[0], 179, lambda x: None)
            cv2.createTrackbar("H High", "Mask Tuner", self._hsv[1], 179, lambda x: None)
            cv2.createTrackbar("S Low",  "Mask Tuner", self._hsv[2], 255, lambda x: None)
            cv2.createTrackbar("S High", "Mask Tuner", self._hsv[3], 255, lambda x: None)
            cv2.createTrackbar("V Low",  "Mask Tuner", self._hsv[4], 255, lambda x: None)
            cv2.createTrackbar("V High", "Mask Tuner", self._hsv[5], 255, lambda x: None)

        self._started = False

    def start(self):
        if not self._started:
            self.picam2.start()
            # small warmup
            cv2.waitKey(300)
            self._started = True

    def stop(self):
        if self._started:
            self.picam2.stop()
            self._started = False

    def close(self):
        self.stop()
        cv2.destroyAllWindows()

    def _get_hsv_bounds(self):
        if self.show_tuner:
            h_low  = cv2.getTrackbarPos("H Low",  "Mask Tuner")
            h_high = cv2.getTrackbarPos("H High", "Mask Tuner")
            s_low  = cv2.getTrackbarPos("S Low",  "Mask Tuner")
            s_high = cv2.getTrackbarPos("S High", "Mask Tuner")
            v_low  = cv2.getTrackbarPos("V Low",  "Mask Tuner")
            v_high = cv2.getTrackbarPos("V High", "Mask Tuner")
            self._hsv = [h_low, h_high, s_low, s_high, v_low, v_high]
        h_low, h_high, s_low, s_high, v_low, v_high = self._hsv
        lower = np.array([h_low, s_low, v_low], dtype=np.uint8)
        upper = np.array([h_high, s_high, v_high], dtype=np.uint8)
        return lower, upper

    def read(self, return_images=False):
        """
        Grabs one frame, finds the most prominent circle in the HSV mask,
        and returns:
            result = {
                "x": x_px, "y": y_px, "r": r_px,
                "u": u_norm, "v": v_norm,     # normalized center offsets [-1..1]
                "timestamp": time.time()
            }
        If nothing found, returns None.

        If return_images=True, also returns (frame_bgr, mask) for display.
        """
        frame_rgb = self.picam2.capture_array()
        hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

        lower, upper = self._get_hsv_bounds()
        mask = cv2.inRange(hsv, lower, upper)
        blurred = cv2.GaussianBlur(mask, self.blur_ksize, self.blur_sigma)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_minDist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.minRadius,
            maxRadius=self.maxRadius
        )

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            # choose the largest circle (most likely the ball)
            x, y, r = max(circles, key=lambda c: c[2])

            # draw for visualization
            cv2.circle(frame_bgr, (x, y), r, (0, 255, 0), 2)
            cv2.circle(frame_bgr, (x, y), 3, (0, 0, 255), -1)
            cv2.drawMarker(frame_bgr, (self.cx, self.cy), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=12, thickness=1)

            # normalized offsets for control (center = 0, left = -1, right = +1)
            u = (x - self.cx) / (self.W / 2.0)
            v = (y - self.cy) / (self.H / 2.0)

            result = {
                "x": int(x),
                "y": int(y),
                "r": int(r),
                "u": float(u),
                "v": float(v),
                "timestamp": time.time(),
            }
        else:
            result = None

        if return_images:
            return result, frame_bgr, mask
        else:
            return result
    def show_image(self):

        res, frame_bgr, mask = self.read(return_images=True)
        if res is not None:
            x, y, r = res["x"], res["y"], res["r"]
            cv2.putText(frame_bgr, f"x={x} y={y} r={r}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Ball Tracking", frame_bgr)
        if self.show_tuner:
            cv2.imshow("Mask", mask)

    def run_demo(self):
        """Optional: live view with 'q' to quit."""
        self.start()
        while True:
            res, frame_bgr, mask = self.read(return_images=True)
            if res is not None:
                x, y, r = res["x"], res["y"], res["r"]
                cv2.putText(frame_bgr, f"x={x} y={y} r={r}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow("Ball Tracking", frame_bgr)
            if self.show_tuner:
                cv2.imshow("Mask", mask)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        self.close()


# ---------------------------
# Minimal usage example
# ---------------------------
if __name__ == "__main__":
    tracker = BallTracker(show_tuner=True)
    tracker.start()
    #tracker.run_demo()
    try:
        while True:
            res = tracker.read()  # <-- returns dict or None
            if res is not None:
                # Use res["x"], res["y"], res["r"] or normalized res["u"], res["v"]
                print(res)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        tracker.close()
