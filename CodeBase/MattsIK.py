import numpy as np


# --- Geometry helpers ---
def generate_equilateral_triangle(radius):
    """
    Generate 3 points in a triangle, equally spaced around a circle of given radius.
    Used for both the base and the platform attachment points.
    """
    angles = [0, 2 * np.pi / 3, 4 * np.pi / 3]
    return np.array([[radius * np.cos(a), radius * np.sin(a), 0] for a in angles])

def rotation_matrix(roll, pitch, yaw=0):
    """
    Create a 3D rotation matrix from roll, pitch, and yaw angles (in radians).
    Used to tilt the platform in space.
    """
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx


class ThreeRRSRobot:
    """
    Models a 3-legged (RRS) parallel robot platform.
    Handles forward and inverse kinematics for the ball-balancing platform.
    """
    def __init__(self, d, e, f, g,
                 h_min=100, h_max=400,
                 roll_max_deg=20,
                 pitch_max_deg=20,
                 motor_limits=None):
        # d: base triangle radius, e: platform triangle radius
        # f: length of first link, g: length of second link
        self.d, self.e, self.f, self.g = d, e, f, g
        self.h_min, self.h_max = h_min, h_max  # Allowed height range
        self.roll_max = np.deg2rad(roll_max_deg)  # Max roll (radians)
        self.pitch_max = np.deg2rad(pitch_max_deg)  # Max pitch (radians)
        # Motor angle limits for each leg
        if motor_limits is None:
            self.motor_limits = [(-np.pi/2, np.pi/2)] * 3
        else:
            self.motor_limits = motor_limits
        # Calculate base and platform joint positions
        self.base_joints = generate_equilateral_triangle(d)
        self.platform_joints = generate_equilateral_triangle(e)

    def forward_platform_joints(self, h, roll, pitch, yaw=0):
        """
        Given a height and orientation, compute the 3D positions of the platform's joints.
        Used internally for kinematics.
        """
        R = rotation_matrix(roll, pitch, yaw)
        return np.array([R @ pj + np.array([0, 0, h]) for pj in self.platform_joints])

    def check_safety(self, h, roll, pitch):
        """
        Check if the requested height and angles are within safe limits.
        Raises ValueError if any limit is exceeded.
        """
        if not (self.h_min <= h <= self.h_max):
            raise ValueError(f"Height {h} mm is out of safe range [{self.h_min}, {self.h_max}]")
        if abs(roll) > self.roll_max:
            raise ValueError(f"Roll {np.rad2deg(roll):.2f}° exceeds safe limit ±{np.rad2deg(self.roll_max)}°")
        if abs(pitch) > self.pitch_max:
            raise ValueError(f"Pitch {np.rad2deg(pitch):.2f}° exceeds safe limit ±{np.rad2deg(self.pitch_max)}°")

    def inverse_kinematics(self, h, roll, pitch, yaw=0):
        """
        Given a desired platform pose (height, roll, pitch), compute the required motor angles.
        Returns two lists: active motor angles and passive joint angles (in radians).
        Raises ValueError if any pose is unreachable or unsafe.
        """
        self.check_safety(h, roll, pitch)
        platform_positions = self.forward_platform_joints(h, roll, pitch, yaw)
        active_thetas = []
        passive_phis = []
        for i in range(3):
            B = self.base_joints[i]  # Base joint position
            P = platform_positions[i]  # Platform joint position
            vec = P - B  # Vector from base to platform
            dist = np.linalg.norm(vec)  # Distance between joints
            # Check if the leg can reach this distance
            if not (abs(self.f - self.g) <= dist <= (self.f + self.g)):
                raise ValueError(f"Leg {i+1} unreachable: dist={dist:.2f} not in [{abs(self.f - self.g)}, {self.f + self.g}]")
            # Compute the required motor angle for this leg
            theta = self._compute_motor_angle(dist, i)
            active_thetas.append(theta)
            # Compute the passive (elbow) joint angle
            phi = self._compute_passive_angle(dist)
            passive_phis.append(phi)
        return active_thetas, passive_phis

    def _compute_motor_angle(self, dist, idx):
        """
        Compute the required motor angle (theta) for a given leg, given the distance between joints.
        Clamps to the allowed range for that motor.
        """
        cos_theta = (self.f ** 2 + dist ** 2 - self.g ** 2) / (2 * self.f * dist)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        theta = np.arccos(cos_theta)
        min_angle, max_angle = self.motor_limits[idx]
        if not (min_angle <= theta <= max_angle):
            theta = min(max(theta, min_angle), max_angle)
        return theta

    def _compute_passive_angle(self, dist):
        """
        Compute the passive (elbow) joint angle for a given leg.
        """
        cos_phi = (self.f ** 2 + self.g ** 2 - dist ** 2) / (2 * self.f * self.g)
        cos_phi = np.clip(cos_phi, -1.0, 1.0)
        return np.arccos(cos_phi)
