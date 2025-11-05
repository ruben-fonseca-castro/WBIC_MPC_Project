import mujoco
import numpy as np

from .lcm2mujuco_bridge import Lcm2MujocoBridge
from arc_bridge.lcm_msgs import unitree_a1_state_t, unitree_a1_control_t
from arc_bridge.utils import *


class UnitreeA1Bridge(Lcm2MujocoBridge):
    def __init__(self, mj_model, mj_data, config):
        super().__init__(mj_model, mj_data, config)

    def parse_robot_specific_low_state(self):
        """Add robot-specific state information to low_state message"""
        # Example: Add inertia matrix and bias forces
        temp_inertia_mat = np.zeros((self.mj_model.nv, self.mj_model.nv))
        mujoco.mj_fullM(self.mj_model, temp_inertia_mat, self.mj_data.qM)
        self.low_state.inertia_mat = temp_inertia_mat.tolist()
        self.low_state.bias_force = self.mj_data.qfrc_bias.tolist()