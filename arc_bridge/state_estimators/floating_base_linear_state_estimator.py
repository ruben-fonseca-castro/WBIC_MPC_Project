import numpy as np
from scipy.linalg import expm
from .kalman_filter import KalmanFilter

class FloatingBaseLinearStateEstimator(KalmanFilter):

    def __init__(self, dt, Q, R, height_init, num_legs=4):
        dim_state = 6   # (px, py, pz, vx, vy, vz)
        dim_control = 3 # (ax, ay, az)
        dim_obs = 6     # Full state observation (px, py, pz, vx, vy, vz)

        Ac = np.zeros((dim_state, dim_state))
        Ac[:3, 3:] = np.eye(3)
        Bc = np.zeros((dim_state, dim_control))
        Bc[3:] = np.eye(3)

        A = expm(dt * Ac)
        B = dt * Bc
        C = np.eye(dim_obs)  # Full state observation

        self.x_init = np.array([0, 0, height_init, 0, 0, 0])
        self.P_init = np.eye(dim_state) * 1e-5
        super().__init__(A, B, C, Q, R, self.x_init, self.P_init)

        # Foot contact tracking for position observation
        self.num_legs = num_legs
        self.foot_touchdown_world = np.zeros((num_legs, 3))  # World position at touchdown
        self.prev_contact = np.zeros(num_legs)  # Previous contact state

    def update_foot_contact(self, contact_state, foot_pos_world):
        """
        Track foot touchdown positions. Call this each timestep.

        Args:
            contact_state: [num_legs] array, 1 = stance, 0 = swing
            foot_pos_world: [num_legs, 3] array, current foot positions in world frame
        """
        for i in range(self.num_legs):
            # Detect touchdown event (transition from swing to stance)
            if contact_state[i] > 0 and self.prev_contact[i] == 0:
                # Store foot world position at touchdown
                self.foot_touchdown_world[i] = foot_pos_world[i].copy()

        self.prev_contact = np.array(contact_state)

    def compute_body_position_from_contacts(self, contact_state, foot_pos_body, R_body_to_world):
        """
        Compute body position in world frame using stance feet.

        For each stance foot:
            p_body_world = p_foot_touchdown_world - R_body_to_world @ p_foot_body

        Returns average across all stance feet, or None if no feet in contact.
        """
        positions = []
        for i in range(self.num_legs):
            if contact_state[i] > 0:
                # Infer body position from this foot
                p_body = self.foot_touchdown_world[i] - R_body_to_world @ foot_pos_body[i]
                positions.append(p_body)

        if len(positions) > 0:
            return np.mean(positions, axis=0)
        return None

    def get_position(self):
        """Returns estimated position [px, py, pz]"""
        return self.x[:3].copy()

    def get_velocity(self):
        """Returns estimated velocity [vx, vy, vz]"""
        return self.x[3:].copy()

