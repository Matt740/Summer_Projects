# file: wrist_ik_controller.py
import numpy as np

class WristIKController:
    """
    3-RRS spherical wrist inverse kinematics (symmetric 120° layout).
    Usage:
        ctrl = WristIKController(L1=0.20, L2=0.30, Rb=0.10, Rp=0.05)
        q1, q2 = ctrl.compute_ik(pitch_deg=15, roll_deg=10, yaw_deg=0)
    """

    def __init__(self, L1: float, L2: float, Rb: float, Rp: float,
                 elbow_mode: str = "up"):
        """
        Args:
            L1, L2: Link lengths
            Rb, Rp: Base/platform radii
            elbow_mode: "up" or "down" for second joint configuration
        """
        self.L1, self.L2 = float(L1), float(L2)
        self.Rb, self.Rp = float(Rb), float(Rp)
        self.elbow_mode = elbow_mode.lower()
        if self.elbow_mode not in ("up", "down"):
            raise ValueError("elbow_mode must be 'up' or 'down'")

        # Precompute symmetric geometry
        self._B = []   # base joint positions in base frame
        self._P0 = []  # platform joint positions in platform frame
        self._e0 = []  # zero-angle link1 directions (radial)
        beta0, gamma0 = 0.0, 0.0
        for i in range(3):
            beta = beta0 + 2*np.pi*i/3
            gamma = gamma0 + 2*np.pi*i/3
            self._B.append([self.Rb*np.cos(beta), self.Rb*np.sin(beta), 0.0])
            self._P0.append([self.Rp*np.cos(gamma), self.Rp*np.sin(gamma), 0.0])
            self._e0.append([np.cos(beta), np.sin(beta), 0.0])
        self._B = np.array(self._B)
        self._P0 = np.array(self._P0)
        self._e0 = np.array(self._e0)
        self._u = np.array([0.0, 0.0, 1.0])  # axis of first revolute joint

    def compute_ik(self, pitch_deg: float, roll_deg: float, yaw_deg: float = 0.0):
        """
        Compute IK for given pitch(y), roll(x), yaw(z) in degrees.
        Returns:
            q1 (np.ndarray): active joint angles [rad]
            q2 (np.ndarray): passive elbow joint angles [rad]
        """
        phi   = np.deg2rad(roll_deg)    # roll about x-axis
        theta = np.deg2rad(pitch_deg)   # pitch about y-axis
        psi   = np.deg2rad(yaw_deg)     # yaw about z-axis

        R = self._rot_z(psi) @ self._rot_y(theta) @ self._rot_x(phi)
        q1_list, q2_list = [], []

        for i in range(3):
            Pi = R @ self._P0[i]        # platform point in base frame
            d = Pi - self._B[i]         # vector from base joint to platform point
            d_perp = d - np.dot(d, self._u) * self._u  # project onto plane ⟂ z

            # q1: angle in perpendicular plane
            num = np.dot(d_perp, np.cross(self._u, self._e0[i]))
            den = np.dot(d_perp, self._e0[i])
            q1_i = np.arctan2(num, den)

            # elbow point after first link rotation
            e1_dir = self._rot_z(q1_i) @ self._e0[i]
            Ei = self._B[i] + self.L1 * e1_dir

            # distance from elbow to platform point
            ell = np.linalg.norm(Pi - Ei)

            # q2: law of cosines
            cos_q2 = (self.L1**2 + self.L2**2 - ell**2) / (2 * self.L1 * self.L2)
            cos_q2 = np.clip(cos_q2, -1.0, 1.0)
            if self.elbow_mode == "up":
                q2_i = np.pi - np.arccos(cos_q2)
            else:
                q2_i = np.pi + np.arccos(cos_q2)

            q1_list.append(q1_i)
            q2_list.append(q2_i)

        return np.array(q1_list), np.array(q2_list)

    @staticmethod
    def _rot_x(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[1, 0, 0],
                         [0, ca, -sa],
                         [0, sa, ca]])

    @staticmethod
    def _rot_y(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[ca, 0, sa],
                         [0, 1, 0],
                         [-sa, 0, ca]])

    @staticmethod
    def _rot_z(a):
        ca, sa = np.cos(a), np.sin(a)
        return np.array([[ca, -sa, 0],
                         [sa,  ca, 0],
                         [0,   0,  1]])


if __name__ == "__main__":
    # Example usage
    ctrl = WristIKController(L1=0.20, L2=0.30, Rb=0.10, Rp=0.05)
    q1, q2 = ctrl.compute_ik(pitch_deg=15, roll_deg=10, yaw_deg=0)

    for i in range(3):
        print(f"Limb {i+1}: q1 = {np.rad2deg(q1[i]):.2f} deg, q2 = {np.rad2deg(q2[i]):.2f} deg")
