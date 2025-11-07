import mujoco
import numpy as np

# Import the state estimator and filter
from arc_bridge.state_estimators import FloatingBaseLinearStateEstimator, MovingWindowFilter
from .lcm2mujuco_bridge import Lcm2MujocoBridge
from arc_bridge.lcm_msgs import unitree_a1_state_t, unitree_a1_control_t
from arc_bridge.utils import *


class UnitreeA1Bridge(Lcm2MujocoBridge):
    def __init__(self, mj_model, mj_data, config):
        super().__init__(mj_model, mj_data, config)

        # --- Robot Specific Names (from a1.xml) ---
        self.torso_name = "trunk"
        self.num_legs = 4
        
        # Body Link Names (for kinematic calculations)
        self.foot_link_names = [
            "FR_calf", 
            "FL_calf", 
            "RR_calf", 
            "RL_calf"
        ]
        
        # Geom/Site Names (for contact sensors and foot position)
        # Note: You used 'FR_foot_site', etc., in the XML setup
        self.foot_geom_names = [
            "FR_foot_site", 
            "FL_foot_site", 
            "RR_foot_site", 
            "RL_foot_site"
        ]

        # --- Joint Offsets ---
        # Assuming controller's 'zero' is the same as the XML's 'zero'
        # Adjust these if your controller expects a different home pose (e.g., 'home' keyframe)
        self.joint_offsets = np.zeros(12)

        # --- State Estimator ---
        self.height_init = 0.43 # From trunk body pos in a1.xml
        
        # Process noise (px, py, pz, vx, vy, vz)
        KF_Q = np.diag([0.002, 0.002, 0.002, 0.02, 0.02, 0.02])
        # Measurement noise (pz, vx, vy, vz)
        KF_R = np.diag([0.001, 1, 1, 50])
        
        self.KF = FloatingBaseLinearStateEstimator(self.config.dt_sim, KF_Q, KF_R, self.height_init)
        self.low_state.position = [0, 0, self.height_init]
        self.low_state.quaternion = [1, 0, 0, 0] # wxyz
        self.low_cmd.contact = [1, 1, 1, 1] # Start assuming all 4 feet on ground
        
        self.foot_radius = 0.02 # From a1.xml <geom class="foot" size="0.02">
        self.gravity = np.array([0, 0, -9.81])
        
        # Hip positions relative to trunk origin (from a1.xml)
        self.hip_pos_body_frame = np.array([
            [ 0.183, -0.047, 0], # FR
            [ 0.183,  0.047, 0], # FL
            [-0.183, -0.047, 0], # RR
            [-0.183,  0.047, 0]  # RL
        ])

        # --- Contact Estimation (Ported from tron1) ---
        # Note: You'll need to adapt this if you use Pinocchio
        # self.P_hat = np.zeros(self.pin_model.nv) # estimated generalized momentum
        # self.Ko = 100 # observer gain
        # self.contact_threshold = -4
        # self.selection_mat = np.eye(self.pin_model.nv, self.num_motor, k=-6)

        # --- Signal Smoothing ---
        self.se_filter = MovingWindowFilter(window_size=10, dim=6)

        # --- Visualization ---
        self.vis_se = True # override default flag
        self.vis_pos_est = np.array([0, 0, self.height_init])
        self.vis_vel_est = np.zeros(3)
        self.vis_R_body = np.eye(3)
        self.vis_box_size = [0.25, 0.1, 0.1] # Adjusted box for A1 trunk

    def update_state_estimation(self):
        """
        Estimates floating base position and velocity using a Kalman Filter.
        This function is copied from tron1 bridge and needs 
        'calculate_foot_position_and_velocity' to be implemented for A1.
        """
        # Retrive states from IMU readings
        omega_body = self.low_state.omega
        acc_body = self.low_state.acceleration
        R_body_to_world = quat_to_rot(Quaternion(*self.low_state.quaternion))

        # Predict based on accelerations
        acc_world = R_body_to_world @ acc_body
        se_state = self.KF.predict(acc_world + self.gravity)

        # Calculate foot positions and velocities in the BODY frame
        pf, vf = self.calculate_foot_position_and_velocity() # You must implement this!

        # Correct based on foot contact
        for idx in range(self.num_legs):
            if self.low_cmd.contact[idx] > 0:
                foot_vel_body = vf[idx]
                vel_measured = -R_body_to_world @ (foot_vel_body + np.cross(omega_body, pf[idx]))
                height_measured = -(R_body_to_world @ pf[idx])[2] + self.foot_radius
                se_state = self.KF.correct(np.append(height_measured, vel_measured))

        se_state_smoothed = self.se_filter.calculate_average(se_state)
        self.vis_pos_est = se_state_smoothed[:3]
        self.vis_vel_est = se_state_smoothed[3:]
        self.vis_R_body = R_body_to_world

        # Write estimated states into low_state
        self.low_state.position[2] = se_state_smoothed[2]
        self.low_state.velocity[:] = se_state_smoothed[3:]

        if self.low_cmd.reset_se:
            self.KF.reset(np.array([0, 0, self.height_init, 0, 0, 0]))

    def calculate_foot_position_and_velocity(self):
        """
        Calculates the position and velocity of all 4 feet relative to the 
        trunk (body) frame using the current joint positions (qj_pos) and 
        velocities (qj_vel).

        Use self.low_state.qj_pos (12-dim) and self.low_state.qj_vel (12-dim)
        
        Returns:
            pf (np.array(4, 3)): Foot positions in body frame [FR, FL, RR, RL]
            vf (np.array(4, 3)): Foot velocities in body frame [FR, FL, RR, RL]
        """
        
        l1 = 0.08505 # abad to hip
        l2 = 0.2   # hip to knee
        l3 = 0.2   # knee to foot

        qj_pos_np = np.array(self.low_state.qj_pos).reshape((4, 3))
        qj_vel_np = np.array(self.low_state.qj_vel).reshape((4, 3))
        th1, th2, th3 = qj_pos_np[:, 0], qj_pos_np[:, 1], qj_pos_np[:, 2]
        dth1, dth2, dth3 = qj_vel_np[:, 0], qj_vel_np[:, 1], qj_vel_np[:, 2]

        p_foot = np.array([-l1 - l3*np.sin(th2 + th3) - l2*np.sin(th2),
                           np.sin(th1)*(l3*np.cos(th2 + th3) + l2*np.cos(th2)),
                           -np.cos(th1)*(l3*np.cos(th2 + th3) + l2*np.cos(th2))]).T

        v_foot = np.array([-l2*dth2*np.cos(th2) - l3*dth2*np.cos(th2 + th3) - l3*dth3*np.cos(th2 + th3),
                           l2*dth1*np.cos(th1)*np.cos(th2) 
                           - l2*dth2*np.sin(th1)*np.sin(th2) 
                           + l3*dth1*np.cos(th1)*np.cos(th2)*np.cos(th3) 
                           - l3*dth1*np.cos(th1)*np.sin(th2)*np.sin(th3) 
                           - l3*dth2*np.cos(th2)*np.sin(th1)*np.sin(th3) 
                           - l3*dth2*np.cos(th3)*np.sin(th1)*np.sin(th2) 
                           - l3*dth3*np.cos(th2)*np.sin(th1)*np.sin(th3) 
                           - l3*dth3*np.cos(th3)*np.sin(th1)*np.sin(th2),
                           l2*dth1*np.sin(th1)*np.cos(th2) 
                           + l2*dth2*np.cos(th1)*np.sin(th2) 
                           + l3*dth1*np.sin(th1)*np.cos(th2)*np.cos(th3) 
                           + l3*dth2*np.cos(th1)*np.cos(th2)*np.sin(th3) 
                           + l3*dth2*np.cos(th1)*np.cos(th3)*np.sin(th2) 
                           + l3*dth3*np.cos(th1)*np.cos(th2)*np.sin(th3) 
                           + l3*dth3*np.cos(th1)*np.cos(th3)*np.sin(th2) 
                           - l3*dth1*np.sin(th1)*np.sin(th2)*np.sin(th3)]).T

        return p_foot + self.hip_pos_body_frame, v_foot

    def parse_robot_specific_low_state(self, backend="mujoco"):
        """
        This function now mirrors the tron1 bridge's structure.
        1. Updates state estimation (KF)
        2. Updates all kinematics/dynamics (Jacobians, etc.)
        """
        
        # Parse common robot states to low_state first
        # (This is handled in the base class 'parse_common_low_state')

        # Update low_state.position[2] and low_state.velocity
        self.update_state_estimation()

        self.update_kinematics_and_dynamics_mujoco()

    def update_kinematics_and_dynamics_mujoco(self):
        """
        Fills the low_state LCM message with MuJoCo-derived physics data
        (inertia, bias, Jacobians, etc.)
        """
        
        # --- Inertia Matrix (H) and Bias Forces (C) ---
        temp_inertia_mat = np.zeros((self.mj_model.nv, self.mj_model.nv))
        mujoco.mj_fullM(self.mj_model, temp_inertia_mat, self.mj_data.qM)
        self.low_state.inertia_mat = temp_inertia_mat.tolist()
        self.low_state.bias_force = self.mj_data.qfrc_bias.tolist()

        # --- Jacobians ---
        # dq = [v_world, omega_body, qj_vel]
        dq = np.concatenate((self.low_state.velocity, self.low_state.omega, self.low_state.qj_vel), axis=0)
        
        # --- Torso Jacobians (J_tor, dJdq_tor) ---
        torso_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.torso_name)
        torso_pos = self.mj_data.xpos[torso_id]
        
        J_tor_trans = np.zeros((3, self.mj_model.nv))
        J_tor_rot = np.zeros((3, self.mj_model.nv))
        dJ_tor_trans = np.zeros((3, self.mj_model.nv))
        dJ_tor_rot = np.zeros((3, self.mj_model.nv))

        mujoco.mj_jac(self.mj_model, self.mj_data, J_tor_trans, J_tor_rot, torso_pos, torso_id)
        mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_tor_trans, dJ_tor_rot, torso_pos, torso_id)

        # Stack to shape (12, nv) as required by your LCM message (pad with zeros)
        J_tor = np.zeros((12, self.mj_model.nv))
        J_tor[0:3, :] = J_tor_trans
        J_tor[3:6, :] = J_tor_rot
        # The remaining rows (6:12) are left as zeros

        dJ_tor = np.zeros((12, self.mj_model.nv))
        dJ_tor[0:3, :] = dJ_tor_trans
        dJ_tor[3:6, :] = dJ_tor_rot
        # The remaining rows (6:12) are left as zeros

        self.low_state.J_tor = J_tor.tolist()
        self.low_state.dJdq_tor = (dJ_tor @ dq).tolist()

        # self.low_state.J_tor = J_tor.flatten().tolist()
        # self.low_state.dJdq_tor = (dJ_tor @ dq).flatten().tolist()

        # --- Foot Jacobians (J_gc, dJdq_gc) and Positions (p_gc) ---
        J_gc_list = []
        dJdq_gc_list = []
        p_gc_list = []

        for i in range(self.num_legs):
            link_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.foot_link_names[i])
            geom_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SITE, self.foot_geom_names[i])
            foot_pos = self.mj_data.site_xpos[geom_id]
            
            J_foot_lin = np.zeros((3, self.mj_model.nv))
            dJ_foot_lin = np.zeros((3, self.mj_model.nv))
            
            mujoco.mj_jac(self.mj_model, self.mj_data, J_foot_lin, None, foot_pos, link_id)
            mujoco.mj_jacDot(self.mj_model, self.mj_data, dJ_foot_lin, None, foot_pos, link_id)
            
            dJdq_foot_lin = dJ_foot_lin @ dq
            
            J_gc_list.append(J_foot_lin)
            dJdq_gc_list.append(dJdq_foot_lin)
            p_gc_list.append(foot_pos)

        # Concatenate for all 4 feet
        self.low_state.J_gc = np.concatenate(J_gc_list, axis=0).tolist()
        self.low_state.dJdq_gc = np.concatenate(dJdq_gc_list, axis=0).tolist()
        self.low_state.p_gc = np.concatenate(p_gc_list, axis=0).tolist()

    def lcm_state_handler(self, channel, data):
        """
        Handles incoming state messages to *set* the simulation state.
        This is useful for resetting the sim to a specific controller state.
        (Copied from tron1 and adapted for A1's 12 joints)
        """
        if self.mj_data is None:
            return

        msg = eval(self.topic_state+"_t").decode(data)
        
        # Set simulation state
        self.mj_data.qpos[0] = msg.position[0]
        self.mj_data.qpos[1] = msg.position[1]
        self.mj_data.qpos[2] = self.low_state.position[2] # Use estimated height
        self.mj_data.qpos[3:7] = msg.quaternion[:]
        self.mj_data.qpos[7 : 7 + 12] = msg.qj_pos[:] # XML and controller are aligned
        self.mj_data.qvel[:] = 0 # Reset velocity

        # Partially update low_state for the *next* tick's estimation
        # This keeps the bridge's state in sync with the reset
        self.low_state.qj_pos[:] = (np.array(msg.qj_pos) + self.joint_offsets).tolist()
        self.low_state.qj_vel[:] = msg.qj_vel
        self.low_state.qj_tau[:] = msg.qj_tau
        self.low_state.acceleration[:] = msg.acceleration
        self.low_state.omega[:] = msg.omega
        self.low_state.quaternion[:] = msg.quaternion
        self.low_state.rpy[:] = msg.rpy