import time
import numpy as np
import scipy.sparse as sparse
import osqp
import lcm
import math

from arc_bridge.lcm_msgs import unitree_a1_state_t, mpc_plan_t, xbox_command_t
from utils import build_Ak, build_Bk
from gait import get_gait_schedule, get_stance_trajectory, get_swing_trajectory_bezier, get_footstep_target

class MPCNode:
    def __init__(self):
        self.lc = lcm.LCM()
        
        self.state_topic = "unitree_a1_state"
        self.joy_topic = "XBOX_COMMAND"
        self.plan_topic = "unitree_a1_mpc_plan"
        
        self.lc.subscribe(self.state_topic, self.state_handler)
        self.lc.subscribe(self.joy_topic, self.joy_handler)
        
        self.state_msg = None
        self.joy_msg = None
        
        self.MASS = 12.45
        self.GRAVITY = 9.81
        
        HIP_OFFSET_Y = 0.047
        THIGH_OFFSET_Y = 0.08505
        WIDTH_Y = HIP_OFFSET_Y + THIGH_OFFSET_Y
        LENGTH_X = 0.183
        
        self.p_shoulders_body = np.array([
            [ LENGTH_X, -WIDTH_Y, 0],
            [ LENGTH_X,  WIDTH_Y, 0],
            [-LENGTH_X, -WIDTH_Y, 0],
            [-LENGTH_X,  WIDTH_Y, 0]
        ]).T
        
        self.gait_trot = {
            'T_cycle': 0.30,
            'stance_percent': 0.55,
            'phase_offsets': [0.0, 0.5, 0.5, 0.0]
        }
        
        self.gait_walk = {
            'T_cycle': 1.5,
            'stance_percent': 0.80,
            'phase_offsets': [0.0, 0.5, 0.25, 0.75]
        }
        
        self.current_gait = self.gait_trot
        
        self.k_raibert = 0.03
        self.swing_height = 0.06
        self.cmd_body_height = 0.3
        
        self.USE_JOYSTICK = True
        self.CMD_VEL_X = 0.3
        self.CMD_VEL_Y = 0.0
        self.CMD_YAW_RATE = 0.0
        
        self.mpc_freq = 40
        self.dt = 1.0 / self.mpc_freq
        self.N_horizon = 20
        self.MU = 0.6
        
        self.Q_stand = np.diag([200, 200, 10, 20, 20, 50, 10, 10, 0.5, 3, 3, 5])
        self.Q_loco = np.diag([375, 375, 25, 30, 40, 40, 15, 15, 0.5, 3, 3, 5])
        
        R_leg_xy = 1e-9
        R_leg_z = 1e-10
        self.R = np.diag(np.tile([R_leg_xy, R_leg_xy, R_leg_z], 4))
        
        self.FSM_STAND = 0
        self.FSM_LOCOMOTION = 1
        self.current_fsm_state = self.FSM_STAND
        
        I_body = np.diag([0.0159, 0.0378, 0.0457])
        self.I_body_inv = np.linalg.inv(I_body)
        self.g_vec = np.array([0, 0, -self.GRAVITY])
        self.g_hat_vec = np.concatenate([np.zeros(9), self.g_vec * self.dt])
        
        self.is_initialized = False
        self.gait_timer = 0.0
        self.current_cmd_pos = np.zeros(3)
        self.current_cmd_yaw = 0.0
        self.foot_pos_start = np.zeros((3, 4))
        self.debug_cnt = 0
        
        self.prev_contact_state = np.ones(4)
        self.touchdown_positions = np.zeros((3, 4))
        self.leg_phase_timers = np.zeros(4)
        self.standing_foot_positions = np.zeros((3, 4))
        
        # OSQP workspace — rebuilt each call for now (warm-start added below)
        self.osqp_prob = None
        self.osqp_n_vars = None   # detect when problem size changes (stand vs loco)
        
    def state_handler(self, channel, data):
        self.state_msg = unitree_a1_state_t.decode(data)
        
    def joy_handler(self, channel, data):
        self.joy_msg = xbox_command_t.decode(data)
        
    def run_mpc(self):
        if self.state_msg is None:
            return
            
        state = self.state_msg
        joy = self.joy_msg
        
        if not self.is_initialized:
            self.current_cmd_pos = np.array(state.position)
            self.current_cmd_pos[2] = self.cmd_body_height
            self.current_cmd_yaw = state.rpy[2]
            self.foot_pos_start = np.array(state.p_gc).reshape((4, 3)).T
            self.touchdown_positions = np.copy(self.foot_pos_start)
            self.standing_foot_positions = np.copy(self.foot_pos_start)
            self.is_initialized = True
            print("MPC Initialized")
            
        # Joy
        # When USE_JOYSTICK=True but no controller is connected (joy=None),
        # default to ZERO velocity so the robot stays in STAND mode.
        if self.USE_JOYSTICK:
            if joy is not None:
                move_req = (joy.left_stick_y < -0.1) or (abs(joy.left_stick_x) > 0.1) or (abs(joy.right_stick_x) > 0.1)
                v_des_body = np.zeros(3)
                des_yaw_rate = 0.0
                if move_req:
                    v_des_body = np.array([-joy.left_stick_y/2, -joy.right_stick_x/2, 0])
                    des_yaw_rate = -joy.left_stick_x
            else:
                # No joystick connected — hold still
                move_req = False
                v_des_body = np.zeros(3)
                des_yaw_rate = 0.0
        else:
            move_req = (abs(self.CMD_VEL_X) > 0.01) or (abs(self.CMD_VEL_Y) > 0.01)
            v_des_body = np.array([self.CMD_VEL_X, self.CMD_VEL_Y, 0.0])
            des_yaw_rate = self.CMD_YAW_RATE
            
        # FSM
        if self.current_fsm_state == self.FSM_STAND:
            if move_req:
                self.current_fsm_state = self.FSM_LOCOMOTION
        elif self.current_fsm_state == self.FSM_LOCOMOTION:
            time_left = self.current_gait['T_cycle'] - self.gait_timer
            if not move_req and (time_left < self.dt * 2):
                self.current_fsm_state = self.FSM_STAND
                self.current_cmd_pos = np.array(state.position)
                self.current_cmd_pos[2] = self.cmd_body_height
                self.gait_timer = 0.0
                
        yaw = state.rpy[2]
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw),  math.cos(yaw), 0],
            [0, 0, 1]
        ])
        v_des_world = R_z @ v_des_body
        body_omega_cmd = np.array([0, 0, des_yaw_rate])
        des_pitch = 0.0
        
        if self.current_fsm_state == self.FSM_STAND:
            self.current_cmd_pos[2] = self.cmd_body_height
            contact_cmd = np.ones(4)
        else:
            self.gait_timer = (self.gait_timer + self.dt) % self.current_gait['T_cycle']
            self.current_cmd_pos[0:2] = np.array(state.position)[0:2] + v_des_world[0:2] * self.dt
            self.current_cmd_pos[2] = self.cmd_body_height
            self.current_cmd_yaw += des_yaw_rate * self.dt
            
        body_pos_cmd = np.copy(self.current_cmd_pos)
        body_vel_cmd = np.copy(v_des_world)
        body_rpy_cmd = np.array([0, des_pitch, self.current_cmd_yaw])
        
        if self.current_fsm_state == self.FSM_STAND:
            Q = self.Q_stand
        else:
            Q = self.Q_loco
            
        R_yaw = R_z
        foot_pos_cmd_world = np.zeros(12)
        foot_vel_cmd_world = np.zeros(12)
        
        stance_delta = 0.02
        
        if self.current_fsm_state == self.FSM_STAND:
            self.standing_foot_positions = np.array(state.p_gc).reshape((4, 3)).T
            foot_pos_cmd_world = self.standing_foot_positions.flatten('F')
            contact_cmd = np.ones(4)
        else:
            contact_cmd = np.zeros(4)
            for i in range(4):
                contact, swing_phase = get_gait_schedule(
                    self.gait_timer, self.current_gait['T_cycle'], 
                    self.current_gait['stance_percent'], self.current_gait['phase_offsets'][i]
                )
                contact_cmd[i] = contact
                
                if contact != self.prev_contact_state[i]:
                    self.leg_phase_timers[i] = 0
                    if contact == 1:
                        self.touchdown_positions[:, i] = np.array(state.p_gc)[i*3:i*3+3]
                else:
                    self.leg_phase_timers[i] += self.dt
                    
                if contact == 1:
                    T_stance = self.current_gait['T_cycle'] * self.current_gait['stance_percent']
                    stance_phase = min(self.leg_phase_timers[i] / T_stance, 1.0)
                    p_stance, v_stance = get_stance_trajectory(
                        self.touchdown_positions[:, i], stance_phase, T_stance, stance_delta
                    )
                    foot_pos_cmd_world[i*3:i*3+3] = p_stance
                    foot_vel_cmd_world[i*3:i*3+3] = v_stance
                else:
                    p_shoulder = np.array(state.position) + R_yaw @ self.p_shoulders_body[:, i]
                    p_target = get_footstep_target(
                        np.array(state.velocity), v_des_world, body_omega_cmd,
                        p_shoulder, self.current_gait['T_cycle'] * self.current_gait['stance_percent'],
                        self.k_raibert, self.cmd_body_height, self.GRAVITY
                    )
                    p_target[2] = self.touchdown_positions[2, i]
                    T_swing = self.current_gait['T_cycle'] * (1 - self.current_gait['stance_percent'])
                    p_swing, v_swing = get_swing_trajectory_bezier(
                        self.touchdown_positions[:, i], p_target, swing_phase, self.swing_height, T_swing
                    )
                    foot_pos_cmd_world[i*3:i*3+3] = p_swing
                    foot_vel_cmd_world[i*3:i*3+3] = v_swing
                    
            self.prev_contact_state = np.copy(contact_cmd)
            
        # MPC QP Formulation
        x_current = np.concatenate([state.rpy, state.position, state.omega, state.velocity])
        x_size = 12
        u_size = 12
        n_vars = (self.N_horizon + 1) * (x_size + u_size)
        x_ref_traj = np.zeros((x_size, self.N_horizon + 1))
        contact_plan = np.zeros((4, self.N_horizon + 1))
        
        for k in range(self.N_horizon + 1):
            t_pred = k * self.dt
            ref_rpy = body_rpy_cmd + body_omega_cmd * t_pred
            
            if self.current_fsm_state == self.FSM_LOCOMOTION:
                ref_pos = np.array(state.position) + body_vel_cmd * t_pred
                ref_pos[2] = self.cmd_body_height
                ref_rpy[2] = state.rpy[2] + body_omega_cmd[2] * t_pred
            else:
                ref_pos = body_pos_cmd + body_vel_cmd * t_pred
                
            x_ref_traj[:, k] = np.concatenate([ref_rpy, ref_pos, body_omega_cmd, body_vel_cmd])
            
            if self.current_fsm_state == self.FSM_STAND:
                contact_plan[:, k] = np.ones(4)
            else:
                t_future = (self.gait_timer + t_pred) % self.current_gait['T_cycle']
                for leg in range(4):
                    c, _ = get_gait_schedule(
                        t_future, self.current_gait['T_cycle'], 
                        self.current_gait['stance_percent'], self.current_gait['phase_offsets'][leg]
                    )
                    contact_plan[leg, k] = c
                    
        # Sparse Matrix formulation
        H = sparse.lil_matrix((n_vars, n_vars))
        f_vec = np.zeros(n_vars)
        
        n_eq_rows = n_vars
        A_eq = sparse.lil_matrix((n_eq_rows, n_vars))
        b_eq = np.zeros(n_eq_rows)
        
        A_ineq_list = []
        b_ineq_list = []
        
        def x_idx(k): return slice(k*x_size, (k+1)*x_size)
        def u_idx(k): return slice((self.N_horizon+1)*x_size + k*u_size, (self.N_horizon+1)*x_size + (k+1)*u_size)
        
        for k in range(self.N_horizon + 1):
            H[x_idx(k), x_idx(k)] = Q
            if k < self.N_horizon:
                H[u_idx(k), u_idx(k)] = self.R
            
            f_vec[x_idx(k)] = -Q @ x_ref_traj[:, k]
            
            ref_yaw_k = x_ref_traj[2, k]
            ref_pos_k = x_ref_traj[3:6, k]
            Ak = build_Ak(ref_yaw_k, self.dt)
            p_feet_mat = foot_pos_cmd_world.reshape((4, 3)).T
            Bk = build_Bk(self.MASS, self.I_body_inv, ref_yaw_k, ref_pos_k, p_feet_mat, self.dt, contact_plan[:, k])
            
            if k == 0:
                A_eq[x_idx(k), x_idx(k)] = np.eye(12)
                A_eq[x_idx(k), u_idx(k)] = -Bk
                b_eq[x_idx(k)] = Ak @ x_current + self.g_hat_vec
            else:
                A_eq[x_idx(k), x_idx(k-1)] = -Ak
                A_eq[x_idx(k), x_idx(k)] = np.eye(12)
                if k < self.N_horizon:
                    A_eq[x_idx(k), u_idx(k)] = -Bk
                b_eq[x_idx(k)] = self.g_hat_vec
                
            if k < self.N_horizon:
                for leg in range(4):
                    idx_leg_start = u_idx(k).start + leg*3
                    idx_leg = slice(idx_leg_start, idx_leg_start+3)
                    
                    if contact_plan[leg, k] == 0:
                        # SWING: zero ALL three force components.
                        # MATLAB: constrains only Fz via two rows [1;-1]*Fz <= 0
                        # which enforces Fz=0. We replicate that exactly:
                        #   row 0: +Fz <= 0
                        #   row 1: -Fz <= 0  → combined: Fz = 0
                        # Fx, Fy are left free (MPC will naturally drive them to 0
                        # via the R cost; hard-constraining them causes infeasibility).
                        A_sub = np.zeros((2, n_vars))
                        fz_col = idx_leg_start + 2          # column of Fz
                        A_sub[0, fz_col] =  1.0             #  Fz <= 0
                        A_sub[1, fz_col] = -1.0             # -Fz <= 0  → Fz = 0
                        A_ineq_list.append(A_sub)
                        b_ineq_list.extend([0, 0])
                    else:
                        cone_mat = np.array([
                            [ 1,  0, -self.MU],
                            [-1,  0, -self.MU],
                            [ 0,  1, -self.MU],
                            [ 0, -1, -self.MU]
                        ])
                        A_sub = np.zeros((5, n_vars))
                        A_sub[0:4, idx_leg] = cone_mat
                        A_sub[4, idx_leg_start+2] = -1.0 # -Fz <= -min_force
                        A_ineq_list.append(A_sub)
                        b_ineq_list.extend([0, 0, 0, 0, -8.0])
                        
        A_ineq = np.vstack(A_ineq_list) if len(A_ineq_list) > 0 else np.zeros((0, n_vars))
        b_ineq = np.array(b_ineq_list)
        
        # Combine equality and inequality constraints for OSQP
        A_osqp = sparse.vstack([A_eq, sparse.csc_matrix(A_ineq)], format='csc')
        l_osqp = np.concatenate([b_eq, -np.inf * np.ones(b_ineq.shape)])
        u_osqp = np.concatenate([b_eq, b_ineq])

        H_csc = H.tocsc()
        n_ineq = A_ineq.shape[0]
        prob_key = (n_vars, n_ineq)  # fingerprint: changes when stance leg count changes

        if self.osqp_prob is None or self.osqp_n_vars != prob_key:
            # Full setup on first call or when problem structure changes
            self.osqp_prob = osqp.OSQP()
            self.osqp_prob.setup(
                H_csc, f_vec, A_osqp, l_osqp, u_osqp,
                verbose=False,
                eps_abs=1e-5, eps_rel=1e-5,
                max_iter=4000,
                warm_starting=True,
                polish=True,
                adaptive_rho=True,
            )
            self.osqp_n_vars = prob_key
        else:
            # Cheap update: reuse symbolic analysis, refresh numeric data only
            self.osqp_prob.update(q=f_vec, l=l_osqp, u=u_osqp)

        res = self.osqp_prob.solve()
        
        if res.info.status_val == 1:
            reaction_force_cmd = res.x[u_idx(0)]
        else:
            reaction_force_cmd = np.zeros(12)
            reaction_force_cmd[2::3] = self.MASS * self.GRAVITY / 4
            
        # Publish
        plan_msg = mpc_plan_t()
        plan_msg.timestamp = state.timestamp
        plan_msg.contact = contact_cmd.tolist()
        plan_msg.reaction_force = reaction_force_cmd.tolist()
        plan_msg.body_pos_cmd = body_pos_cmd.tolist()
        plan_msg.body_rpy_cmd = body_rpy_cmd.tolist()
        plan_msg.body_vel_cmd = body_vel_cmd.tolist()
        plan_msg.body_omega_cmd = body_omega_cmd.tolist()
        plan_msg.foot_pos_cmd = foot_pos_cmd_world.tolist()
        plan_msg.foot_vel_cmd = foot_vel_cmd_world.tolist()
        
        self.lc.publish(self.plan_topic, plan_msg.encode())

    def run(self):
        print("Running Python MPC Node...")
        try:
            while True:
                self.lc.handle_timeout(1)
                self.run_mpc()
                time.sleep(self.dt)
        except KeyboardInterrupt:
            print("Stopped.")

if __name__ == "__main__":
    node = MPCNode()
    node.run()
