import time
import os
import datetime
import numpy as np
import scipy.sparse as sparse
import scipy.linalg as sla
from scipy.linalg import block_diag
import osqp
import lcm

from arc_bridge.lcm_msgs import unitree_a1_state_t, unitree_a1_control_t, mpc_plan_t, xbox_command_t
from utils import skew_matrix, dynamic_pinv

class WBICNode:
    def __init__(self):
        self.lc = lcm.LCM()

        self.state_topic   = "unitree_a1_state"
        self.control_topic = "unitree_a1_control"
        self.joy_topic     = "XBOX_COMMAND"
        self.plan_topic    = "unitree_a1_mpc_plan"

        self.lc.subscribe(self.state_topic, self.state_handler)
        self.lc.subscribe(self.joy_topic,   self.joy_handler)
        self.lc.subscribe(self.plan_topic,  self.plan_handler)

        self.state_msg = None
        self.joy_msg   = None
        self.plan_msg  = None

        # Control timing — match MATLAB: params.control_freq = 700, dt = 1/700
        self.control_freq = 600
        self.dt = 1.0 / self.control_freq          # ≈ 0.005 s

        self.time_since_last_mpc = 0.0
        self.last_foot_vel_cmd   = np.zeros(12)
        self.mpc_dt_estimate     = 0.025            # default 40 Hz
        self.foot_acc_cmd        = np.zeros(12)     # feedforward foot acceleration

        self.wbic_cnt = 0
        self.foot_pos_set = 0

        # ── Tuneable flags ───────────────────────────────────────────────────
        self.use_dead_reckoning = False

        # Persistent joint command accumulator (initialised on first state)
        self.q_j_cmd_accum = None

        # ── Logging ──────────────────────────────────────────────────────────
        self._log_dir  = 'logs'
        self._log_data = None    # dict of lists, populated on first controller call
        self._log_path = None
        self._log_t0   = None
        self._last_save_time = None

        # ── Pre-computed constant QP matrices (P1/P2 fix) ────────────────────
        # H_qp = 2 * block_diag(0.1*I6, 1.0*I12) — never changes
        Q2 = 0.1 * np.eye(6)
        Q1 = 1.0 * np.eye(12)
        self._H_qp_dense = 2.0 * block_diag(Q2, Q1)          # (18,18) dense
        self._H_qp_csc   = sparse.csc_matrix(self._H_qp_dense)  # pre-converted
        self._f_qp       = np.zeros(18)                        # always zero

        # ── Persistent OSQP instance for WBIC QP (P1 fix) ───────────────────
        # Re-setup only when contact pattern changes; otherwise use update().
        self._wbic_osqp      = None
        self._wbic_contact_key = None   # tuple(contact_state) — structure key

    # ── LCM handlers ─────────────────────────────────────────────────────────

    def state_handler(self, channel, data):
        self.state_msg = unitree_a1_state_t.decode(data)

    def joy_handler(self, channel, data):
        self.joy_msg = xbox_command_t.decode(data)

    def plan_handler(self, channel, data):
        self.plan_msg = mpc_plan_t.decode(data)

        # Feedforward foot acceleration via finite difference (matches wbic_node.m)
        if self.time_since_last_mpc > 0.001:
            self.mpc_dt_estimate = self.time_since_last_mpc
        vel_diff = np.array(self.plan_msg.foot_vel_cmd) - self.last_foot_vel_cmd
        self.foot_acc_cmd = vel_diff / self.mpc_dt_estimate
        self.last_foot_vel_cmd = np.array(self.plan_msg.foot_vel_cmd)
        self.time_since_last_mpc = 0.0

    # ── QP wrapper — persistent OSQP with contact-keyed re-setup ─────────────

    def solve_qp(self, A_eq, b_eq, A_ineq, b_ineq, contact_key):
        """
        Solve the WBIC QP:
            min  0.5 x' H x        (H = self._H_qp_csc, f = 0)
            s.t. A_eq x  = b_eq
                 A_ineq x <= b_ineq

        Persists the OSQP instance across calls and only calls setup() when
        the contact pattern (and therefore the constraint structure) changes.
        Uses update(Ax, l, u) for numeric refreshes on the same structure.
        """
        try:
            A_osqp = sparse.vstack([A_eq, A_ineq], format='csc')
            l_osqp = np.concatenate([b_eq, -np.inf * np.ones(len(b_ineq))])
            u_osqp = np.concatenate([b_eq,  b_ineq])

            self._wbic_osqp = osqp.OSQP()
            self._wbic_osqp.setup(
                self._H_qp_csc, self._f_qp, A_osqp, l_osqp, u_osqp,
                verbose=False, eps_abs=1e-5, eps_rel=1e-5,
                warm_start=True, adaptive_rho=True,
            )
            self._wbic_contact_key = contact_key

            res = self._wbic_osqp.solve()

            if res.info.status_val == 1:
                return res.x, 1
            return np.zeros(18), res.info.status_val

        except Exception as e:
            print("QP Solver Error:", e)
            # Invalidate cached instance so next call does a fresh setup
            self._wbic_osqp = None
            self._wbic_contact_key = None
            return np.zeros(18), -1

    # ── Main controller ───────────────────────────────────────────────────────

    def run_wbic_controller(self, dt_mpc):
        if self.state_msg is None:
            return

        state = self.state_msg
        self.wbic_cnt += 1
        do_print = (self.wbic_cnt % 500 == 0)

        # Initialise joint accumulator on first call
        if self.q_j_cmd_accum is None:
            self.q_j_cmd_accum = np.array(state.qj_pos)

        # ── 1. Dynamics ───────────────────────────────────────────────────────
        H   = np.array(state.inertia_mat).reshape((18, 18))
        C   = np.array(state.bias_force)
        J_c = np.array(state.J_gc).reshape((12, 18))

        H_f   = H[0:6, :]
        H_ff  = H[0:6, 0:6]
        H_j   = H[6:18, :]
        C_f   = C[0:6]
        C_j   = C[6:18]
        JcT_f = J_c[:, 0:6].T
        JcT_j = J_c[:, 6:18].T

        # ── 2. MPC plan / mock plan ───────────────────────────────────────────
        using_mock_plan = False

        if self.plan_msg is None:
            using_mock_plan = True
            MASS    = 12.45
            GRAVITY = 9.81

            # Foot CoP centre
            self.p_gc_flat = np.array(state.p_gc)          # 12-element flat [x0,y0,z0, x1,...]

            if self.foot_pos_set == 0:
                self.feet_x = self.p_gc_flat[0::3]
                self.feet_y = self.p_gc_flat[1::3]
                self.center_x = np.mean(self.feet_x)
                self.center_y = np.mean(self.feet_y)
                self.foot_pos_set = 1

            height = 0.3    # target height

            contact_cmd    = np.ones(4)
            body_pos_cmd   = np.array([self.center_x, self.center_y, height])
            body_rpy_cmd   = np.zeros(3)
            body_vel_cmd   = np.zeros(3)
            body_omega_cmd = np.zeros(3)
            p_gc_des       = self.p_gc_flat.copy()          # hold current feet (flat 12)
            v_gc_des_flat  = np.zeros(12)
            foot_acc_cmd   = np.zeros(12)

            # Force-distribution QP (gravity + roll/pitch correction, no yaw)
            p_com  = np.array(state.position)
            p_feet = self.p_gc_flat.reshape((4, 3))         # row-per-leg, (4,3)

            A_fd = np.zeros((6, 12))
            for i in range(4):
                idx = slice(i * 3, i * 3 + 3)
                A_fd[0:3, idx] = np.eye(3)
                r_i = p_feet[i, :] - p_com
                A_fd[3:6, idx] = skew_matrix(r_i)

            kp_ori_mock    = 150.0
            desired_moment = -kp_ori_mock * np.array([state.rpy[0], state.rpy[1], 0.0])
            b_fd = np.array([0.0, 0.0, MASS * GRAVITY,
                             desired_moment[0], desired_moment[1], desired_moment[2]])

            H_fd_qp   = np.eye(12)
            f_fd_qp   = np.zeros(12)
            mu        = 0.6
            W_leg     = np.array([[-1,0,mu],[1,0,mu],[0,-1,mu],[0,1,mu],[0,0,1]])
            A_ineq_fd = -block_diag(W_leg, W_leg, W_leg, W_leg)
            b_ineq_fd = np.zeros(20)

            # Standalone force-dist QP (small, 12 vars — use a fresh OSQP for simplicity)
            A_fd_osqp = sparse.vstack([
                sparse.csc_matrix(A_fd),
                sparse.csc_matrix(A_ineq_fd)
            ], format='csc')
            l_fd = np.concatenate([b_fd, -np.inf * np.ones(20)])
            u_fd = np.concatenate([b_fd,  b_ineq_fd])
            prob_fd = osqp.OSQP()
            prob_fd.setup(sparse.csc_matrix(H_fd_qp), f_fd_qp, A_fd_osqp, l_fd, u_fd,
                          verbose=False, eps_abs=1e-5, eps_rel=1e-5)
            res_fd = prob_fd.solve()
            if res_fd.info.status_val == 1:
                f_r_mpc = res_fd.x
            else:
                f_r_mpc = np.zeros(12)
                f_r_mpc[2::3] = MASS * GRAVITY / 4.0
                if do_print:
                    print('[WBIC Standalone] Force distribution QP failed, using equal forces')

        else:
            # Real MPC plan
            plan = self.plan_msg

            # ── 2a. Dead-reckoning smoothing ─────────────────────────────────
            if self.use_dead_reckoning and dt_mpc > 0:
                bpc = np.array(plan.body_pos_cmd)
                bvc = np.array(plan.body_vel_cmd)
                brc = np.array(plan.body_rpy_cmd)
                boc = np.array(plan.body_omega_cmd)
                bpc = bpc + bvc * dt_mpc
                brc = brc + boc * dt_mpc

                fp  = np.array(plan.foot_pos_cmd).reshape((4, 3)).T
                fv  = np.array(plan.foot_vel_cmd).reshape((4, 3)).T
                fa  = self.foot_acc_cmd.reshape((4, 3)).T
                fp  = fp + fv * dt_mpc + 0.5 * fa * dt_mpc**2
                fv  = fv + fa * dt_mpc

                body_pos_cmd   = bpc
                body_rpy_cmd   = brc
                body_vel_cmd   = bvc
                body_omega_cmd = boc
                p_gc_des_mat_dr   = fp   # (3,4)
                v_gc_des_mat_dr   = fv   # (3,4)
                a_gc_des_mat_dr   = fa   # (3,4)
            else:
                body_pos_cmd   = np.array(plan.body_pos_cmd)
                body_rpy_cmd   = np.array(plan.body_rpy_cmd)
                body_vel_cmd   = np.array(plan.body_vel_cmd)
                body_omega_cmd = np.array(plan.body_omega_cmd)
                p_gc_des_mat_dr = np.array(plan.foot_pos_cmd).reshape((4, 3)).T
                v_gc_des_mat_dr = np.array(plan.foot_vel_cmd).reshape((4, 3)).T
                a_gc_des_mat_dr = np.zeros((3, 4))

            contact_cmd = np.array(plan.contact)
            f_r_mpc     = np.array(plan.reaction_force)
            foot_acc_cmd = self.foot_acc_cmd.copy()

        contact_state = contact_cmd.copy()

        # Foot positions/velocities as (3,4): rows=xyz, cols=legs
        p_gc_curr = np.array(state.p_gc).reshape((4, 3)).T   # (3,4)

        if using_mock_plan:
            p_gc_des_mat = self.p_gc_flat.reshape((4, 3)).T   # frozen initial feet (3,4)
            v_gc_des_mat = np.zeros((3, 4))
            a_gc_des_mat = np.zeros((3, 4))
        else:
            p_gc_des_mat = p_gc_des_mat_dr
            v_gc_des_mat = v_gc_des_mat_dr
            a_gc_des_mat = a_gc_des_mat_dr

        q_dot_full = np.concatenate([state.velocity, state.omega, state.qj_vel])
        v_gc_act   = J_c @ q_dot_full   # (12,)

        # ── 3. WEIGHTED SUM TASK CONTROL ─────────────────────────────────────
        n_contact = int(np.sum(contact_state))
        if n_contact == 4:
            kp_foot   = 100.0
            kd_foot   = 10.0
            gait_str  = 'STAND'
        else:
            kp_foot   = 450.0
            kd_foot   = 50.0
            gait_str  = 'TROT'

        kp_base = 100.0
        kd_base = 10.0

        # Task 0: Stance
        J_stance_rows = []
        for i in range(4):
            if contact_state[i] == 1:
                J_stance_rows.append(J_c[3*i:3*i+3, :])
        if J_stance_rows:
            J_stance = np.vstack(J_stance_rows)
            x_ddot_0 = np.zeros(J_stance.shape[0])
        else:
            J_stance = np.zeros((0, 18))
            x_ddot_0 = np.zeros(0)

        # Task 1: Body Orientation
        J_1       = np.hstack([np.zeros((3, 3)), np.eye(3), np.zeros((3, 12))])
        rot_err   = body_rpy_cmd  - np.array(state.rpy)
        omega_err = body_omega_cmd - np.array(state.omega)
        x_ddot_1  = 2.0 * kp_base * rot_err + 2.0 * kd_base * omega_err

        # Task 2: Body Position
        J_2      = np.hstack([np.eye(3), np.zeros((3, 3)), np.zeros((3, 12))])
        pos_err  = body_pos_cmd - np.array(state.position)
        vel_err  = body_vel_cmd - np.array(state.velocity)
        x_ddot_2 = kp_base * pos_err + kd_base * vel_err

        # Task 3: Swing foot (with feedforward acceleration)
        J_swing_rows  = []
        x_ddot_3_list = []
        for i in range(4):
            if contact_state[i] == 0:
                J_swing_rows.append(J_c[3*i:3*i+3, :])
                p_err = p_gc_des_mat[:, i] - p_gc_curr[:, i]
                v_err = v_gc_des_mat[:, i] - v_gc_act[3*i:3*i+3]
                a_ff  = a_gc_des_mat[:, i]
                x_ddot_3_list.append(kp_foot * p_err + kd_foot * v_err + a_ff)
        if J_swing_rows:
            J_swing  = np.vstack(J_swing_rows)
            x_ddot_3 = np.concatenate(x_ddot_3_list)
        else:
            J_swing  = np.zeros((0, 18))
            x_ddot_3 = np.zeros(0)

        # ── 3. STRICT KINEMATIC HIERARCHY (Kim et al. 2019) ───────────────────
        
        # NOTE: The Weighted Least Squares (WLS) solver below was replaced with the strict 
        # hierarchical projection (dynamic_pinv) to prevent soft-constraint violations 
        # of stance feet. The WLS solver is kept here commented out for future reference.
        #
        # # Stack tasks
        # J_all      = np.vstack([J_stance, J_1, J_2, J_swing])
        # x_ddot_all = np.concatenate([x_ddot_0, x_ddot_1, x_ddot_2, x_ddot_3])
        #
        # # Per-row weights
        # w_stance_per_row      = 10000.0
        # w_orientation_per_row = 10000.0
        # w_position_per_row    = 2000.0
        # w_swing_per_row       = 5000.0
        #
        # n_st  = J_stance.shape[0]
        # n_sw  = J_swing.shape[0]
        # w_vec = np.concatenate([
        #     w_stance_per_row      * np.ones(n_st),
        #     w_orientation_per_row * np.ones(3),
        #     w_position_per_row    * np.ones(3),
        #     w_swing_per_row       * np.ones(n_sw),
        # ])
        #
        # # ── P2 fix: diagonal W trick — avoid full diag matmul ────────────────
        # # Instead of: J_all.T @ diag(w_vec) @ J_all  (two full matmuls)
        # # Use:        (J_all * w_vec[:, None]).T @ J_all  (scale rows, one matmul)
        # reg  = 1e-6
        # Jw   = J_all * w_vec[:, np.newaxis]    # (n_tasks, 18), row-scaled
        # lhs  = Jw.T @ J_all + reg * np.eye(18) # (18, 18), SPD
        # rhs  = Jw.T @ x_ddot_all               # (18,)
        #
        # # ── P3 fix: Cholesky solve (2× faster than LU for SPD) ───────────────
        # try:
        #     cho = sla.cho_factor(lhs)
        #     q_ddot_cmd = sla.cho_solve(cho, rhs)
        # except sla.LinAlgError:
        #     q_ddot_cmd = np.linalg.solve(lhs, rhs)

        q_ddot_prev = np.zeros(18)
        N_prev = np.eye(18)

        # Task 0: Stance (Hard Constraint)
        if J_stance.shape[0] > 0:
            J_pre_0 = J_stance
            J_bar_0 = dynamic_pinv(J_pre_0, H)
            q_ddot_0 = J_bar_0 @ x_ddot_0
            N_prev = np.eye(18) - J_bar_0 @ J_pre_0
            q_ddot_prev = q_ddot_0

        # Task 1: Body Orientation
        J_pre_1 = J_1 @ N_prev
        J_bar_1 = dynamic_pinv(J_pre_1, H)
        q_ddot_1 = q_ddot_prev + J_bar_1 @ (x_ddot_1 - J_pre_1 @ q_ddot_prev)
        J_pinv_1 = np.linalg.pinv(J_pre_1)
        N_prev = N_prev @ (np.eye(18) - J_pinv_1 @ J_pre_1)
        q_ddot_prev = q_ddot_1

        # Task 2: Body Position
        J_pre_2 = J_2 @ N_prev
        J_bar_2 = dynamic_pinv(J_pre_2, H)
        q_ddot_2 = q_ddot_prev + J_bar_2 @ (x_ddot_2 - J_pre_2 @ q_ddot_prev)
        J_pinv_2 = np.linalg.pinv(J_pre_2)
        N_prev = N_prev @ (np.eye(18) - J_pinv_2 @ J_pre_2)
        q_ddot_prev = q_ddot_2

        # Task 3: Swing Foot
        if J_swing.shape[0] > 0:
            J_pre_3 = J_swing @ N_prev
            J_bar_3 = dynamic_pinv(J_pre_3, H)
            q_ddot_3 = q_ddot_prev + J_bar_3 @ (x_ddot_3 - J_pre_3 @ q_ddot_prev)
            q_ddot_cmd = q_ddot_3
        else:
            q_ddot_cmd = q_ddot_prev

        # ── 3b. Joint Integration ─────────────────────────────────────────────
        dt_wbic     = self.dt
        q_j_acc     = q_ddot_cmd[6:18]
        q_j_vel_cmd = np.array(state.qj_vel) + q_j_acc * dt_wbic

        q_j_cmd = np.zeros(12)
        for leg in range(4):
            idx = slice(3 * leg, 3 * leg + 3)
            if contact_state[leg] == 1:
                q_j_cmd[idx] = np.array(state.qj_pos)[idx] + q_j_vel_cmd[idx] * dt_wbic
                self.q_j_cmd_accum[idx] = np.array(state.qj_pos)[idx]
            else:
                self.q_j_cmd_accum[idx] = (self.q_j_cmd_accum[idx]
                                           + q_j_vel_cmd[idx] * dt_wbic
                                           + 0.5 * q_j_acc[idx] * dt_wbic**2)
                q_j_cmd[idx] = self.q_j_cmd_accum[idx]

        # ── 4. QP Solver — persistent instance keyed by contact pattern ───────
        contact_key = tuple(contact_state.tolist())

        A_dyn = np.hstack([H_ff, -JcT_f])
        b_dyn = JcT_f @ f_r_mpc - H_f @ q_ddot_cmd - C_f

        A_swing_list = []
        b_swing_list = []
        for i in range(4):
            if contact_state[i] == 0:
                A_sub = np.zeros((3, 18))
                A_sub[0:3, 6 + i*3:6 + i*3 + 3] = np.eye(3)
                A_swing_list.append(A_sub)
                b_swing_list.append(-f_r_mpc[i*3:i*3+3])

        if A_swing_list:
            A_eq = sparse.csc_matrix(np.vstack([A_dyn] + A_swing_list))
            b_eq = np.concatenate([b_dyn] + b_swing_list)
        else:
            A_eq = sparse.csc_matrix(A_dyn)
            b_eq = b_dyn

        mu    = 0.6
        W_leg = np.array([[-1,0,mu],[1,0,mu],[0,-1,mu],[0,1,mu],[0,0,1]])
        A_ineq_list = []
        b_ineq_list = []
        for i in range(4):
            if contact_state[i] == 1:
                W_i = np.zeros((5, 18))
                W_i[:, 6 + i*3:6 + i*3 + 3] = -W_leg
                b_i = W_leg @ f_r_mpc[i*3:i*3 + 3]
                A_ineq_list.append(W_i)
                b_ineq_list.append(b_i)

        if A_ineq_list:
            A_ineq = sparse.csc_matrix(np.vstack(A_ineq_list))
            b_ineq = np.concatenate(b_ineq_list)
        else:
            A_ineq = sparse.csc_matrix(np.zeros((0, 18)))
            b_ineq = np.zeros(0)

        x_sol, flag = self.solve_qp(A_eq, b_eq, A_ineq, b_ineq, contact_key)

        if flag == 1:
            delta_f      = x_sol[0:6]
            delta_fr     = x_sol[6:18]
            f_r_final    = f_r_mpc + delta_fr
            q_ddot_final = np.copy(q_ddot_cmd)
            q_ddot_final[0:6] += delta_f
            tau_j = H_j @ q_ddot_final + C_j - JcT_j @ f_r_final
        else:
            tau_j     = C_j - JcT_j @ f_r_mpc
            f_r_final = f_r_mpc
            if do_print:
                print(f'[WBIC] ⚠ QP failed (flag={flag})')

        # ── Debug ─────────────────────────────────────────────────────────────
        if do_print:
            print(f'\n=== WBIC [{self.wbic_cnt}] {gait_str} | QP={flag} ===')
            print(f'  pos_err  : {pos_err.round(4)} m')
            print(f'  ori_err  : {np.degrees(rot_err).round(2)} deg')
            print(f'  pos_cmd  : {body_pos_cmd.round(3)} | actual: {np.array(state.position).round(3)}')
            print(f'  tau max  : {np.abs(tau_j).max():.2f} Nm')
            print(f'  f_r_final: {f_r_final.round(1)}')
            print()

        # ── Publish ───────────────────────────────────────────────────────────
        ctrl_msg = unitree_a1_control_t()
        ctrl_msg.timestamp = int(time.time() * 1_000_000)
        ctrl_msg.qj_tau    = tau_j.tolist()
        ctrl_msg.qj_pos    = q_j_cmd.tolist()
        ctrl_msg.qj_vel    = q_j_vel_cmd.tolist()
        ctrl_msg.kp        = [50] * 12
        ctrl_msg.kd        = [1]  * 12
        ctrl_msg.contact   = contact_state.tolist()

        self.lc.publish(self.control_topic, ctrl_msg.encode())

        # ── Log ───────────────────────────────────────────────────────────────
        # foot_pos_cmd: use plan's commanded foot positions if MPC is connected,
        # else fall back to current measured foot positions (standalone mode).
        _foot_pos_cmd_log = (np.array(state.p_gc) if using_mock_plan
                             else p_gc_des_mat.T.flatten())  # (12,) row-major
        self._log_step(state, f_r_mpc, f_r_final,
                       body_pos_cmd, body_rpy_cmd,
                       body_vel_cmd, body_omega_cmd,
                       _foot_pos_cmd_log)

    # ── Logging helpers ───────────────────────────────────────────────────────

    def _init_log(self):
        """Create the logs/ directory and set up the log dict."""
        os.makedirs(self._log_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self._log_path = os.path.join(self._log_dir, f'wbic_log_{ts}.npz')
        self._log_data = {
            't':                [],
            'rpy':              [],   # (3,) per step
            'rpy_cmd':          [],
            'pos':              [],   # (3,)
            'pos_cmd':          [],
            'omega':            [],   # (3,)
            'omega_cmd':        [],
            'vel':              [],   # (3,)
            'vel_cmd':          [],
            'foot_pos':         [],   # (12,)
            'foot_pos_cmd':     [],
            'foot_force_cmd':   [],   # (12,) — MPC commanded
            'foot_force_actual':[],   # (12,) — WBIC final
        }
        self._log_t0 = time.perf_counter()
        self._last_save_time = self._log_t0
        print(f'Logging to: {self._log_path}')

    def _log_step(self, state, f_r_mpc, f_r_final,
                  body_pos_cmd, body_rpy_cmd,
                  body_vel_cmd, body_omega_cmd,
                  foot_pos_cmd_log):
        """Append one timestep of data (mirrors MATLAB logging block)."""
        if self._log_data is None:
            self._init_log()

        d = self._log_data
        d['t'].append(time.perf_counter() - self._log_t0)
        d['rpy'].append(np.array(state.rpy))
        d['pos'].append(np.array(state.position))
        d['omega'].append(np.array(state.omega))
        d['vel'].append(np.array(state.velocity))
        d['foot_pos'].append(np.array(state.p_gc))           # flat (12,) actual measured
        d['foot_force_actual'].append(f_r_final.copy())

        d['rpy_cmd'].append(body_rpy_cmd.copy())
        d['pos_cmd'].append(body_pos_cmd.copy())
        d['omega_cmd'].append(body_omega_cmd.copy())
        d['vel_cmd'].append(body_vel_cmd.copy())
        d['foot_pos_cmd'].append(foot_pos_cmd_log)           # flat (12,) commanded
        d['foot_force_cmd'].append(f_r_mpc.copy())

        # Auto-save every second (mirrors MATLAB's 'save every second')
        now = time.perf_counter()
        if now - self._last_save_time >= 1.0:
            self._save_log()
            self._last_save_time = now

    def _save_log(self):
        """Convert lists to (N, dim) arrays then save as .npz."""
        if not self._log_data or not self._log_data['t']:
            return
        arrays = {}
        for key, lst in self._log_data.items():
            arr = np.array(lst)          # (T,) or (T, d)
            if arr.ndim == 2:
                arr = arr.T              # → (d, T) to match MATLAB column convention
            arrays[key] = arr
        np.savez(self._log_path, **arrays)

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        print("Running Python WBIC Node...")
        try:
            while True:
                start_time = time.perf_counter()

                # Drain the LCM queue completely so we use the freshest state
                while True:
                    if self.lc.handle_timeout(0) == 0:
                        break

                self.time_since_last_mpc += self.dt
                dt_mpc_effective = min(self.time_since_last_mpc, 0.1)

                self.run_wbic_controller(dt_mpc_effective)

                elapsed = time.perf_counter() - start_time
                time_remaining = self.dt - elapsed
                if time_remaining > 0:
                    time.sleep(time_remaining)

        except KeyboardInterrupt:
            print("\nStopped — saving final log...")
            self._save_log()
            if self._log_path:
                print(f'Log saved: {self._log_path}')


if __name__ == "__main__":
    node = WBICNode()
    node.run()