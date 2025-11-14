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

        # print(f"Est Pos-Z: {se_state_smoothed[2]:.3f}  |  Est Vel-Z: {se_state_smoothed[5]:.3f}")

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
        
        # Get trunk (body) orientation and position from MuJoCo
        torso_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.torso_name)
        p_body_world = self.mj_data.xpos[torso_id]
        R_body_to_world = self.mj_data.xmat[torso_id].reshape((3, 3))
        R_world_to_body = R_body_to_world.T

        pf_body_list = []
        vf_body_list = []
        
        # Full (18-dim) velocity vector from MuJoCo
        qvel_full = self.mj_data.qvel 

        for i in range(self.num_legs):
            # Get IDs for this leg
            geom_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SITE, self.foot_geom_names[i])
            link_id = mujoco.mj_name2id(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, self.foot_link_names[i])
            
            # --- 1. Calculate Position ---
            # Get foot position in world frame
            p_foot_world = self.mj_data.site_xpos[geom_id]
            
            # Transform to body frame
            p_foot_body = R_world_to_body @ (p_foot_world - p_body_world)
            pf_body_list.append(p_foot_body)

            # --- 2. Calculate Velocity ---
            # Get (3, nv) linear Jacobian for the foot in the world frame
            J_foot_lin_world = np.zeros((3, self.mj_model.nv))
            mujoco.mj_jac(self.mj_model, self.mj_data, J_foot_lin_world, None, p_foot_world, link_id)
            
            # Get linear velocity of the foot in the world frame
            v_foot_world = J_foot_lin_world @ qvel_full
            
            # Get linear velocity of the *body* in the world frame
            v_body_world = self.mj_data.qvel[0:3]
            
            # Get angular velocity of the *body* in the body frame
            omega_body = self.mj_data.qvel[3:6]
            
            # Transform foot velocity from world to body frame using the transport theorem
            # v_foot_body = R_world_to_body @ (v_foot_world - v_body_world) - np.cross(omega_body, p_foot_body)
            v_foot_body = R_world_to_body @ (v_foot_world - v_body_world) - np.cross(omega_body, p_foot_body)
            vf_body_list.append(v_foot_body)
            
        return np.array(pf_body_list), np.array(vf_body_list)

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

    def lcm_control_handler(self, channel, data):
        """
        Handles incoming control messages from the MATLAB controller.
        
        This function implements the paper's final control law:
        tau_final = tau_pd + tau_ff
        tau_pd = Kp(q_cmd - q) + Kd(q_dot_cmd - q_dot)
        
        This tau_final is then sent to MuJoCo.
        """
        if self.mj_data is None:
            return # Don't run if simulation isn't ready

        try:
            # 1. Decode the message from MATLAB
            msg = unitree_a1_control_t.decode(data)
            
            if msg is None:
                return

            # 2. Get all 3 command components as numpy arrays
            q_j_cmd   = np.array(msg.qj_pos)
            q_j_vel_cmd = np.array(msg.qj_vel)
            tau_ff    = np.array(msg.qj_tau) # Feedforward torque

            # 3. Get the gains from the message
            kp = np.array(msg.kp)
            kd = np.array(msg.kd)

            # 4. Get the *current* state from the simulation
            # (This is populated by your state estimator)
            q_pos_curr = np.array(self.low_state.qj_pos)
            q_vel_curr = np.array(self.low_state.qj_vel)

            # 5. --- IMPLEMENT THE PD CONTROLLER ---
            # Calculate the position and velocity error
            pos_error = q_j_cmd - q_pos_curr
            vel_error = q_j_vel_cmd - q_vel_curr
            
            # Calculate the PD torque
            tau_pd = kp * pos_error + kd * vel_error
            
            # 6. --- THIS IS THE FINAL COMMAND ---
            # Add the feedforward torque from your QP
            tau_final = tau_pd + tau_ff
            
            # 7. Apply the final, total torque to MuJoCo's actuators
            self.mj_data.ctrl[:] = tau_final
            
            # Also store the contact state for the state estimator
            self.low_cmd.contact = msg.contact

        except Exception as e:
            print(f"Error in lcm_control_handler: {e}")