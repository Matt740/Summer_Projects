import numpy as np

class ThreeRRSRobot:
    def __init__(self, d, e, f, g,
                 h_min=100, h_max=400,
                 roll_max=np.deg2rad(20),
                 pitch_max=np.deg2rad(20),
                 motor_limits = [(0.0, np.deg2rad(143))] * 3):
        """
        d : float
            Distance from base center to base corners
        e : float
            Distance from platform center to platform corners
        f : float
            Length of link #1
        g : float
            Length of link #2
        h_min, h_max : float
            Safe vertical range for platform height
        roll_max, pitch_max : float
            Safe tilt range (radians)
        motor_limits : list of (min,max) radians for each motor
        """
        self.d, self.e, self.f, self.g = d, e, f, g
        self.h_min, self.h_max = h_min, h_max
        self.roll_max, self.pitch_max = roll_max, pitch_max
        self.motor_limits = motor_limits if motor_limits else [(-np.pi/2, np.pi/2)]*3

        # Base joint positions (equilateral triangle in XY plane)
        self.base_joints = self._generate_triangle(d)
        # Platform attachment points (equilateral triangle in XY plane)
        self.platform_joints = self._generate_triangle(e)

    def _generate_triangle(self, radius):
        """Generate 3 equidistant points (triangle corners) around z=0."""
        angles = [0, 2*np.pi/3, 4*np.pi/3]
        return np.array([[radius*np.cos(a), radius*np.sin(a), 0] for a in angles])

    def _rotation_matrix(self, roll, pitch, yaw=0):
        """Return rotation matrix from roll, pitch, yaw (ZYX convention)."""
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll),  np.cos(roll)]])
        Ry = np.array([[ np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw),  np.cos(yaw), 0],
                       [0, 0, 1]])
        return Rz @ Ry @ Rx

    def forward_platform_joints(self, h, roll, pitch, yaw=0):
        """
        Compute platform joint positions given height h and orientation (roll, pitch, yaw).
        """
        R = self._rotation_matrix(roll, pitch, yaw)
        return np.array([R @ pj + np.array([0, 0, h]) for pj in self.platform_joints])

    def inverse_kinematics(self, h, roll, pitch, yaw=0):
        """
        Solve IK: Find joint angles (actuated R joints) for each leg given platform pose.
        Returns:
            active_thetas : list of motor angles (radians)
            passive_phis  : list of passive joint (elbow) angles (radians)
        """
        # ---- Safety checks ----
        # if not (self.h_min <= h <= self.h_max):
        #     raise ValueError(f"Height {h} mm is out of safe range [{self.h_min}, {self.h_max}]")
        # if abs(roll) > self.roll_max:
        #     raise ValueError(f"Roll {np.rad2deg(roll):.2f}° exceeds safe limit ±{np.rad2deg(self.roll_max)}°")
        # if abs(pitch) > self.pitch_max:
        #     raise ValueError(f"Pitch {np.rad2deg(pitch):.2f}° exceeds safe limit ±{np.rad2deg(self.pitch_max)}°")

        # Compute transformed platform joints
        platform_positions = self.forward_platform_joints(h, roll, pitch, yaw)
        active_thetas = []
        passive_phis = []

        for i in range(3):
            B = self.base_joints[i]     # Base joint
            P = platform_positions[i]   # Platform joint

            vec = P - B
            dist = np.linalg.norm(vec)

            # Workspace reach check
            if not (abs(self.f - self.g) <= dist <= (self.f + self.g)):
                raise ValueError(f"Leg {i+1} unreachable: dist={dist:.2f} not in [{abs(self.f - self.g)}, {self.f + self.g}]")

            # Active motor angle (base joint angle)
            cos_theta = (self.f**2 + dist**2 - self.g**2) / (2 * self.f * dist)
            cos_theta = np.clip(cos_theta, -1.0, 1.0)
            theta = np.arccos(cos_theta)

            # Motor angle limit check
            min_angle, max_angle = self.motor_limits[i]
            if not (min_angle <= theta <= max_angle):
                raise ValueError(f"Motor {i+1} angle {theta:.2f} rad out of range [{min_angle}, {max_angle}]")

            active_thetas.append(theta)

            # Passive joint angle (elbow, between f and g)
            cos_phi = (self.f**2 + self.g**2 - dist**2) / (2 * self.f * self.g)
            cos_phi = np.clip(cos_phi, -1.0, 1.0)
            phi = np.arccos(cos_phi)
            passive_phis.append(phi)

        return active_thetas, passive_phis


# ================= Example Usage =================
if __name__ == "__main__":
    # Example robot geometry
    d, e, f, g = 116, 116, 50, 219
    robot = ThreeRRSRobot(d, e, f, g,
                          h_min=100, h_max=400,
                          roll_max=np.deg2rad(15),
                          pitch_max=np.deg2rad(15),
                          motor_limits = [(0.0, np.deg2rad(143))] * 3)

    try:
        h = 225
        roll = np.deg2rad(0)
        pitch = np.deg2rad(0)

        # Get both active and passive angles
        motor_angles, passive_angles = robot.inverse_kinematics(h, roll, pitch)

        print("Motor Angles (rad):", motor_angles)
        print("Motor Angles (deg):", np.rad2deg(motor_angles))
        print("Passive Angles (rad):", passive_angles)
        print("Passive Angles (deg):", np.rad2deg(passive_angles))

    except ValueError as e:
        print("SAFETY ERROR:", e)
