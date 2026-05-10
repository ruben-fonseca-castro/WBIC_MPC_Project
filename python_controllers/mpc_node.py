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
        self.joy_topic   = "XBOX_COMMAND"
        self.plan_topic  = "unitree_a1_mpc_plan"

        self.lc.subscribe(self.state_topic, self.state_handler)
        self.lc.subscribe(self.joy_topic,   self.joy_handler)

        self.state_msg = None
        self.joy_msg   = None

        self.MASS    = 12.45
        self.GRAVITY = 9.81

        HIP_OFFSET_Y  = 0.047
        THIGH_OFFSET_Y = 0.08505
        WIDTH_Y  = HIP_OFFSET_Y + THIGH_OFFSET_Y
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

        self.k_raibert    = 0.03
        self.swing_height = 0.06
        self.cmd_body_height = 0.3

        # Joystick hysteresis: require move_req true for N consecutive frames before
        # transitioning to LOCOMOTION (guards against noisy stick at rest).
        self._move_req_count  = 0
        self._MOVE_REQ_FRAMES = 5   # ~83 ms at 60 Hz

        self.USE_JOYSTICK = True
        self.CMD_VEL_X    = 0.3
        self.CMD_VEL_Y    = 0.0
        self.CMD_YAW_RATE = 0.0

        self.mpc_freq   = 60
        self.dt         = 1.0 / self.mpc_freq
        self.N_horizon  = 20
        self.MU         = 0.6

        self.Q_stand = np.diag([200, 200, 10, 20, 20, 50, 10, 10, 0.5, 3, 3, 5])
        self.Q_loco  = np.diag([375, 375, 25, 30, 40, 40, 15, 15, 0.5, 3, 3, 5])

        R_leg_xy = 1e-9
        R_leg_z  = 1e-10
        self.R   = np.diag(np.tile([R_leg_xy, R_leg_xy, R_leg_z], 4))

        self.FSM_STAND      = 0
        self.FSM_LOCOMOTION = 1
        self.current_fsm_state = self.FSM_STAND

        I_body = np.diag([0.0159, 0.0378, 0.0457])
        self.I_body_inv = np.linalg.inv(I_body)
        self.g_vec      = np.array([0, 0, -self.GRAVITY])
        self.g_hat_vec  = np.concatenate([np.zeros(9), self.g_vec * self.dt])

        self.is_initialized    = False
        self.gait_timer        = 0.0
        self.current_cmd_pos   = np.zeros(3)
        self.current_cmd_yaw   = 0.0
        self.foot_pos_start    = np.zeros((3, 4))
        self.debug_cnt         = 0

        self.prev_contact_state    = np.ones(4)
        self.touchdown_positions   = np.zeros((3, 4))
        self.leg_phase_timers      = np.zeros(4)
        self.standing_foot_positions = np.zeros((3, 4))

        # ── QP dimension constants ────────────────────────────────────────────
        x_size = 12
        u_size = 12
        N      = self.N_horizon

        # Variable layout: [x_0..x_N | u_0..u_N]  (N+1 blocks each for simplicity)
        self._x_size  = x_size
        self._u_size  = u_size
        self._n_x     = (N + 1) * x_size   # 252
        self._n_u     = (N + 1) * u_size   # 252  (u_N slot unused but keeps indexing clean)
        self._n_vars  = self._n_x + self._n_u  # 504

        # ── Pre-compute constant cost matrix H (block-diagonal, never changes) ─
        # H = diag(Q, Q, ...[N+1], R, R, ...[N], 0[u_N unused])
        # We build for both Q_stand and Q_loco; swap via update(Px=...) later.
        # For simplicity, always rebuild H when Q switches (rare FSM event).
        self._H_csc_stand = self._build_H_csc(self.Q_stand)
        self._H_csc_loco  = self._build_H_csc(self.Q_loco)

        # ── Pre-compute fixed-structure A_ineq (P1 fix) ───────────────────────
        # Always 4 legs × N steps × 5 rows = 400 rows.
        # Structure (non-zero positions) is constant; only l/u bounds change.
        #   Row layout per (step k, leg i): k*20 + i*5 to k*20 + i*5 + 4
        #   Cols: u_idx(k) + i*3 to i*3+2
        #   Rows 0-3: cone_mat  (friction cone or disabled via l/u)
        #   Row 4   : -Fz       (stance min force or swing Fz=0 via l=u=0)
        cone_mat = np.array([
            [ 1,  0, -self.MU],
            [-1,  0, -self.MU],
            [ 0,  1, -self.MU],
            [ 0, -1, -self.MU],
        ])
        n_ineq = 4 * 5 * N   # = 400
        A_ineq_dense = np.zeros((n_ineq, self._n_vars))
        for k in range(N):
            u_start = self._n_x + k * u_size
            for i in range(4):
                row0   = k * 20 + i * 5
                col_i  = u_start + i * 3
                A_ineq_dense[row0:row0+4, col_i:col_i+3] = cone_mat
                A_ineq_dense[row0+4,      col_i+2]        = -1.0  # -Fz
        self._A_ineq_csc = sparse.csc_matrix(A_ineq_dense)

        # Bound templates — filled each call from contact_plan
        self._l_ineq_template = -np.inf * np.ones(n_ineq)
        self._u_ineq_template = np.zeros(n_ineq)   # cone rows ≤ 0 for stance
        self._n_ineq = n_ineq

        # ── Pre-compute A_eq sparsity (fixed block pattern) ───────────────────
        # A_eq: (N+1)*x_size rows × n_vars cols
        # k=0 : I*x_0  - B0*u_0                      = A0*x_cur + g_hat
        # k>0 : -Ak*x_{k-1} + I*x_k  - Bk*u_k        = g_hat     (k < N)
        # k=N : -AN*x_{N-1} + I*x_N                   = g_hat
        # All blocks are dense 12×12 → sparsity structure is fixed.
        n_eq = (N + 1) * x_size   # = 252
        self._n_eq   = n_eq
        self._A_eq_dense = np.zeros((n_eq, self._n_vars))  # pre-allocated, reused each call

        # ── Persistent OSQP instance (P1 fix) ────────────────────────────────
        self._osqp_prob    = None
        self._osqp_fsm_key = None   # (FSM_STAND / FSM_LOCOMOTION) — rebuild H when Q changes

    # ── Helper: build constant cost matrix for a given Q ─────────────────────

    def _build_H_csc(self, Q):
        N      = self.N_horizon
        n_vars = self._n_vars
        H_dense = np.zeros((n_vars, n_vars))
        for k in range(N + 1):
            xs = k * self._x_size
            H_dense[xs:xs+self._x_size, xs:xs+self._x_size] = Q
        for k in range(N):  # u_N slot stays zero (unused)
            us = self._n_x + k * self._u_size
            H_dense[us:us+self._u_size, us:us+self._u_size] = self.R
        return sparse.csc_matrix(H_dense)

    # ── LCM handlers ─────────────────────────────────────────────────────────

    def state_handler(self, channel, data):
        self.state_msg = unitree_a1_state_t.decode(data)

    def joy_handler(self, channel, data):
        self.joy_msg = xbox_command_t.decode(data)

    # ── Gait contact plan — vectorised over full horizon (P3 fix) ─────────────

    def _compute_contact_plan(self, N, gait_timer, gait):
        """Compute contact_plan (4 × N+1) for the full prediction horizon
        without a per-leg per-step Python loop."""
        T     = gait['T_cycle']
        sp    = gait['stance_percent']
        offsets = np.array(gait['phase_offsets'])   # (4,)
        t_pred  = np.arange(N + 1) * self.dt         # (N+1,)
        t_future = (gait_timer + t_pred) % T          # (N+1,)

        # phase[i, k] = ((t_future[k] / T) + offsets[i]) % 1
        phase = ((t_future[np.newaxis, :] / T) + offsets[:, np.newaxis]) % 1.0  # (4, N+1)
        contact_plan = (phase < sp).astype(float)   # 1=stance, 0=swing
        return contact_plan

    # ── Main MPC solve ────────────────────────────────────────────────────────

    def run_mpc(self):
        if self.state_msg is None:
            return

        state = self.state_msg
        joy   = self.joy_msg

        if not self.is_initialized:
            self.current_cmd_pos = np.array(state.position)
            self.current_cmd_pos[2] = self.cmd_body_height
            self.current_cmd_yaw = state.rpy[2]
            self.foot_pos_start  = np.array(state.p_gc).reshape((4, 3)).T
            self.touchdown_positions   = np.copy(self.foot_pos_start)
            self.standing_foot_positions = np.copy(self.foot_pos_start)
            self.is_initialized  = True
            print("MPC Initialized")

        # Joy
        v_des_body   = np.zeros(3)
        des_yaw_rate = 0.0
        if self.USE_JOYSTICK:
            if joy is not None:
                # Raised dead-zone (0.2) to guard against stick drift at rest
                raw_move = ((joy.left_stick_y < -0.2) or
                            (abs(joy.left_stick_x) > 0.2) or
                            (abs(joy.right_stick_x) > 0.2))
                if raw_move:
                    v_des_body   = np.array([-joy.left_stick_y/2, -joy.right_stick_x/2, 0])
                    des_yaw_rate = -joy.left_stick_x
            else:
                raw_move = False
        else:
            raw_move = (abs(self.CMD_VEL_X) > 0.01) or (abs(self.CMD_VEL_Y) > 0.01)
            if raw_move:
                v_des_body   = np.array([self.CMD_VEL_X, self.CMD_VEL_Y, 0.0])
                des_yaw_rate = self.CMD_YAW_RATE

        # Hysteresis: only assert move_req after _MOVE_REQ_FRAMES consecutive frames
        if raw_move:
            self._move_req_count = min(self._move_req_count + 1, self._MOVE_REQ_FRAMES)
        else:
            self._move_req_count = 0
        move_req = (self._move_req_count >= self._MOVE_REQ_FRAMES)

        # FSM
        if self.current_fsm_state == self.FSM_STAND:
            if move_req:
                self.current_fsm_state = self.FSM_LOCOMOTION
                # Ensure gait_timer starts clean at locomotion entry
                self.gait_timer = 0.0
        elif self.current_fsm_state == self.FSM_LOCOMOTION:
            if not move_req:
                # Transition back to STAND at any point — don't wait for end-of-cycle.
                # Waiting for time_left < dt*2 was a narrow window that could be missed
                # at higher MPC frequencies (60 Hz), leaving it stuck in LOCOMOTION.
                self.current_fsm_state = self.FSM_STAND
                self.current_cmd_pos   = np.array(state.position)
                self.current_cmd_pos[2] = self.cmd_body_height
                self.current_cmd_yaw   = state.rpy[2]   # re-latch yaw to current
                self.gait_timer = 0.0

        yaw = state.rpy[2]
        cy, sy = math.cos(yaw), math.sin(yaw)
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        v_des_world  = R_z @ v_des_body
        body_omega_cmd = np.array([0, 0, des_yaw_rate])
        des_pitch = 0.0

        if self.current_fsm_state == self.FSM_STAND:
            self.current_cmd_pos[2] = self.cmd_body_height
            contact_cmd = np.ones(4)
        else:
            self.gait_timer = (self.gait_timer + self.dt) % self.current_gait['T_cycle']
            self.current_cmd_pos[0:2] = np.array(state.position)[0:2] + v_des_world[0:2] * self.dt
            self.current_cmd_pos[2]   = self.cmd_body_height
            self.current_cmd_yaw     += des_yaw_rate * self.dt

        body_pos_cmd = np.copy(self.current_cmd_pos)
        body_vel_cmd = np.copy(v_des_world)
        body_rpy_cmd = np.array([0, des_pitch, self.current_cmd_yaw])

        Q = self.Q_stand if self.current_fsm_state == self.FSM_STAND else self.Q_loco

        foot_pos_cmd_world = np.zeros(12)
        foot_vel_cmd_world = np.zeros(12)
        stance_delta       = 0.02

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
                    T_stance    = self.current_gait['T_cycle'] * self.current_gait['stance_percent']
                    stance_phase = min(self.leg_phase_timers[i] / T_stance, 1.0)
                    p_stance, v_stance = get_stance_trajectory(
                        self.touchdown_positions[:, i], stance_phase, T_stance, stance_delta
                    )
                    foot_pos_cmd_world[i*3:i*3+3] = p_stance
                    foot_vel_cmd_world[i*3:i*3+3] = v_stance
                else:
                    p_shoulder = np.array(state.position) + R_z @ self.p_shoulders_body[:, i]
                    p_target   = get_footstep_target(
                        np.array(state.velocity), v_des_world, body_omega_cmd,
                        p_shoulder, self.current_gait['T_cycle'] * self.current_gait['stance_percent'],
                        self.k_raibert, self.cmd_body_height, self.GRAVITY
                    )
                    p_target[2] = self.touchdown_positions[2, i]
                    T_swing     = self.current_gait['T_cycle'] * (1 - self.current_gait['stance_percent'])
                    p_swing, v_swing = get_swing_trajectory_bezier(
                        self.touchdown_positions[:, i], p_target, swing_phase, self.swing_height, T_swing
                    )
                    foot_pos_cmd_world[i*3:i*3+3] = p_swing
                    foot_vel_cmd_world[i*3:i*3+3] = v_swing

            self.prev_contact_state = np.copy(contact_cmd)

        # ── MPC QP Formulation ────────────────────────────────────────────────
        x_current = np.concatenate([state.rpy, state.position, state.omega, state.velocity])
        N         = self.N_horizon
        x_size    = self._x_size
        u_size    = self._u_size
        n_x       = self._n_x
        n_vars    = self._n_vars

        # ── Build Ak once (yaw fixed over horizon) ───────────────────────────
        p_feet_mat = foot_pos_cmd_world.reshape((4, 3)).T
        Ak_cur = build_Ak(yaw, self.dt)

        # ── Reference trajectory and contact plan ─────────────────────────────
        x_ref_traj = np.zeros((x_size, N + 1))
        for k in range(N + 1):
            t_pred  = k * self.dt
            ref_rpy = body_rpy_cmd + body_omega_cmd * t_pred
            if self.current_fsm_state == self.FSM_LOCOMOTION:
                ref_pos    = np.array(state.position) + body_vel_cmd * t_pred
                ref_pos[2] = self.cmd_body_height
                ref_rpy[2] = state.rpy[2] + body_omega_cmd[2] * t_pred
            else:
                ref_pos = body_pos_cmd + body_vel_cmd * t_pred
            x_ref_traj[:, k] = np.concatenate([ref_rpy, ref_pos, body_omega_cmd, body_vel_cmd])

        # Contact plan — vectorised (P3 fix)
        if self.current_fsm_state == self.FSM_STAND:
            contact_plan = np.ones((4, N + 1))
        else:
            contact_plan = self._compute_contact_plan(N, self.gait_timer, self.current_gait)

        # ── Build f_vec (gradient) ────────────────────────────────────────────
        f_vec = np.zeros(n_vars)
        for k in range(N + 1):
            xs = k * x_size
            f_vec[xs:xs+x_size] = -Q @ x_ref_traj[:, k]

        # ── Build A_eq using pre-allocated dense array ─────────────────────────
        # Bk is built per-step using contact_plan[:, k] so the dynamics horizon
        # is consistent with the inequality constraint horizon (critical for STAND:
        # all legs in contact every step → Bk identical each step but correct).
        A_eq_dense = self._A_eq_dense
        A_eq_dense[:] = 0.0
        b_eq = np.zeros(self._n_eq)

        for k in range(N + 1):
            xs = k * x_size
            # Identity on x_k
            A_eq_dense[xs:xs+x_size, xs:xs+x_size] = np.eye(x_size)
            # -Ak on x_{k-1}
            if k > 0:
                A_eq_dense[xs:xs+x_size, xs-x_size:xs] = -Ak_cur
            # -Bk on u_k — contact_plan[:, k] gives per-step stance mask
            if k < N:
                us   = n_x + k * u_size
                Bk_k = build_Bk(self.MASS, self.I_body_inv, yaw, np.array(state.position),
                                 p_feet_mat, self.dt, contact_plan[:, k])
                A_eq_dense[xs:xs+x_size, us:us+u_size] = -Bk_k
            # RHS
            if k == 0:
                b_eq[xs:xs+x_size] = Ak_cur @ x_current + self.g_hat_vec
            else:
                b_eq[xs:xs+x_size] = self.g_hat_vec

        A_eq_csc = sparse.csc_matrix(A_eq_dense)

        # ── Build A_ineq bounds from contact_plan (P1 fix: fixed structure) ───
        # l_ineq[row] = -inf always (inequality: A_ineq x ≤ u_ineq)
        # u_ineq per (k, leg):
        #   Stance: rows 0-3 → 0 (cone ≤ 0),  row 4 → -F_min (-8 N)
        #   Swing : rows 0-3 → +inf (disabled), row 4 → 0 and l=0 (Fz=0 equality)
        l_ineq = self._l_ineq_template.copy()
        u_ineq = np.zeros(self._n_ineq)

        for k in range(N):
            for i in range(4):
                row0 = k * 20 + i * 5
                if contact_plan[i, k] == 1:   # stance
                    # rows 0-3: cone ≤ 0 (already 0 from zeros init)
                    u_ineq[row0+4] = -8.0           # -Fz ≤ -8 → Fz ≥ 8 N
                else:                              # swing
                    # rows 0-3: disable friction cone (free Fx, Fy via cost)
                    u_ineq[row0:row0+4] = np.inf
                    # row 4: Fz = 0 via equality l=u=0
                    l_ineq[row0+4] = 0.0
                    u_ineq[row0+4] = 0.0

        # ── Assemble full OSQP problem ─────────────────────────────────────────
        A_osqp = sparse.vstack([A_eq_csc, self._A_ineq_csc], format='csc')
        l_osqp = np.concatenate([b_eq,  l_ineq])
        u_osqp = np.concatenate([b_eq,  u_ineq])

        H_csc = self._H_csc_stand if self.current_fsm_state == self.FSM_STAND else self._H_csc_loco
        fsm_key = self.current_fsm_state

        self._osqp_prob = osqp.OSQP()
        self._osqp_prob.setup(
            P=H_csc, q=f_vec, A=A_osqp, l=l_osqp, u=u_osqp,
            verbose=False,
            eps_abs=1e-5, eps_rel=1e-5,
            max_iter=4000,
            warm_start=True,
            polish=True,
            adaptive_rho=True,
        )
        self._osqp_fsm_key = fsm_key

        res = self._osqp_prob.solve()

        u_start_0 = n_x   # index of u_0 block
        if res.info.status_val == 1:
            reaction_force_cmd = res.x[u_start_0:u_start_0+12]
        else:
            reaction_force_cmd = np.zeros(12)
            reaction_force_cmd[2::3] = self.MASS * self.GRAVITY / 4

        # ── Publish ───────────────────────────────────────────────────────────
        plan_msg = mpc_plan_t()
        plan_msg.timestamp      = state.timestamp
        plan_msg.contact        = contact_cmd.tolist()
        plan_msg.reaction_force = reaction_force_cmd.tolist()
        plan_msg.body_pos_cmd   = body_pos_cmd.tolist()
        plan_msg.body_rpy_cmd   = body_rpy_cmd.tolist()
        plan_msg.body_vel_cmd   = body_vel_cmd.tolist()
        plan_msg.body_omega_cmd = body_omega_cmd.tolist()
        plan_msg.foot_pos_cmd   = foot_pos_cmd_world.tolist()
        plan_msg.foot_vel_cmd   = foot_vel_cmd_world.tolist()

        self.lc.publish(self.plan_topic, plan_msg.encode())

    def run(self):
        print("Running Python MPC Node...")
        try:
            while True:
                deadline = time.perf_counter() + self.dt

                # Drain the LCM queue completely so we use the freshest state
                while True:
                    if self.lc.handle_timeout(0) == 0:
                        break

                self.run_mpc()

                sleep_dur = deadline - time.perf_counter()
                if sleep_dur > 0:
                    time.sleep(sleep_dur)

        except KeyboardInterrupt:
            print("Stopped.")

if __name__ == "__main__":
    node = MPCNode()
    node.run()