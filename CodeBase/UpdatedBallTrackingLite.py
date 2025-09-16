from picamera2 import Picamera2
import cv2
import numpy as np
import time


class BallTrackerLite:

    @staticmethod
    def filter_recent_data(data_list, closeness_thresh=0.05): ##rfilte threshold is % of frame width/height
        """
        Given a list of up to 3 dicts from .read(),
        compare their (u, v) closeness and return the average if all are close.
        If not close, return the most recent.
        """
        if len(data_list) < 3:
            # Not enough data, just return the most recent
            return data_list[-1]
        uvs = [(d['u'], d['v']) for d in data_list]
        # Compute pairwise distances
        d01 = (abs(uvs[0][0] - uvs[1][0]) + abs(uvs[0][1] - uvs[1][1]))
        d12 = (abs(uvs[1][0] - uvs[2][0]) + abs(uvs[1][1] - uvs[2][1]))
        d02 = (abs(uvs[0][0] - uvs[2][0]) + abs(uvs[0][1] - uvs[2][1]))
        if d01 < closeness_thresh and d12 < closeness_thresh and d02 < closeness_thresh:
            # All close: average
            avg = {
                'u': sum(d['u'] for d in data_list) / 3,
                'v': sum(d['v'] for d in data_list) / 3,
                'x': int(sum(d['x'] for d in data_list) / 3),
                'y': int(sum(d['y'] for d in data_list) / 3),
                'r': int(sum(d['r'] for d in data_list) / 3),
                'timestamp': data_list[-1]['timestamp']
            }
            return avg
        else:
            # Not close: return most recent
            return data_list[-1]
    """
    BallTrackerLite uses the PiCamera2 and OpenCV to find a colored ball in the camera image.
    It returns the ball's position in both pixels and normalized coordinates (for control).
    """
    def __init__(
        self,
        resolution=(640, 480),
        hsv_low=(9, 60, 160),     # HSV color range for the ball (default: orange)
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
        # Store camera and detection settings
        self.resolution = resolution
        self.W, self.H = resolution  # Width and height of the image
        self.cx, self.cy = self.W // 2, self.H // 2  # Center of the image
        self.hsv_low = hsv_low  # Lower HSV bound for color threshold
        self.hsv_high = hsv_high  # Upper HSV bound for color threshold
        self.hough_dp = hough_dp  # Hough transform parameter
        self.hough_minDist = hough_minDist  # Minimum distance between circles
        self.hough_param1 = hough_param1  # Hough param1 (Canny high threshold)
        self.hough_param2 = hough_param2  # Hough param2 (center detection threshold)
        self.minRadius = minRadius  # Minimum circle radius
        self.maxRadius = maxRadius  # Maximum circle radius
        self.blur_ksize = blur_ksize  # Gaussian blur kernel size
        self.blur_sigma = blur_sigma  # Gaussian blur sigma
        self._started = False  # Camera started flag
        self._init_camera()  # Set up the camera
        self._set_hsv_arrays(hsv_low, hsv_high)  # Prepare HSV arrays for masking

    def _init_camera(self):
        # Initialize the PiCamera2 and set the resolution
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_video_configuration(main={"size": self.resolution}))

    def _set_hsv_arrays(self, low, high):
        # Convert HSV bounds to numpy arrays for OpenCV
        self.lower = np.array(low, dtype=np.uint8)
        self.upper = np.array(high, dtype=np.uint8)

    def set_hsv(self, low, high):
        """
        Update HSV color thresholds at runtime.
        Use this if you want to change the color the tracker looks for.
        """
        self.hsv_low = low
        self.hsv_high = high
        self._set_hsv_arrays(low, high)

    def start(self):
        # Start the camera stream (call before reading frames)
        if not self._started:
            self.picam2.start()
            # Wait a short time for camera to adjust exposure
            time.sleep(0.3)
            self._started = True

    def stop(self):
        # Stop the camera stream
        if self._started:
            self.picam2.stop()
            self._started = False

    def close(self):
        # Close the camera (alias for stop)
        self.stop()

    def read(self):
        """
        Capture a frame, find the ball, and return its position.
        Returns a dictionary with pixel and normalized coordinates, or None if no ball is found.
        Example output:
        {
          "x": px, "y": px, "r": px,   # Ball center and radius in pixels
          "u": [-1..1], "v": [-1..1],   # Normalized position (for control)
          "timestamp": float             # Time of capture
        }
        """
        # 1. Capture a frame from the camera
        frame_rgb = self._capture_frame()
        # 2. Convert the image to HSV color space (better for color detection)
        hsv = self._to_hsv(frame_rgb)
        # 3. Create a mask to keep only the pixels in the ball's color range
        mask = self._create_mask(hsv)
        # 4. Use Hough Circle Transform to find circles in the mask
        circles = self._detect_circles(mask)
        if circles is None:
            return None  # No ball found
        # 5. Pick the largest detected circle (assume it's the ball)
        x, y, r = self._select_largest_circle(circles)
        # 6. Normalize the ball's position to [-1, 1] for control
        u, v = self._normalize_coords(x, y)
        return {
            "x": int(x),
            "y": int(y),
            "r": int(r),
            "u": float(u),
            "v": float(v),
            "timestamp": time.time()
        }

    def _capture_frame(self):
        # Grab a frame from the camera as a numpy array (RGB)
        return self.picam2.capture_array()

    def _to_hsv(self, frame_rgb):
        # Convert an RGB image to HSV color space
        return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

    def _create_mask(self, hsv):
        # Create a binary mask where the ball's color is white and everything else is black
        mask = cv2.inRange(hsv, self.lower, self.upper)
        # Blur the mask to reduce noise and help circle detection
        mask = cv2.GaussianBlur(mask, (self.blur_ksize, self.blur_ksize) if isinstance(self.blur_ksize, int) else self.blur_ksize, self.blur_sigma)
        return mask

    def _detect_circles(self, mask):
        # Use Hough Circle Transform to find circles in the mask
        return cv2.HoughCircles(
            mask,
            cv2.HOUGH_GRADIENT,
            dp=self.hough_dp,
            minDist=self.hough_minDist,
            param1=self.hough_param1,
            param2=self.hough_param2,
            minRadius=self.minRadius,
            maxRadius=self.maxRadius
        )

    def _select_largest_circle(self, circles):
        # Convert detected circles to integer coordinates
        circles = np.round(circles[0, :]).astype("int")
        # Return the largest circle (by radius)
        return max(circles, key=lambda c: c[2])  # x, y, r

    def _normalize_coords(self, x, y):
        # Convert pixel coordinates to normalized [-1, 1] range (center is 0,0)
        u = (x - self.cx) / (self.W / 2.0)
        v = (y - self.cy) / (self.W / 2.0)
        return u, v
