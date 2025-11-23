clc;
clear;
clear functions; 
format compact;

addpath('utils/');
addpath('controllers/');
addpath('mpc/');
addpath('gait/');
addpath('lcm/');

disp('--- MPC Controller Node (Walking Gait) ---');

%% ========================================================================
%% 1. CONFIGURATION & PARAMETERS
%% ======================================================================== 

MASS = 12.45; 
GRAVITY = 9.81;

% A1 robot geometry (from a1.xml)
HIP_OFFSET_Y = 0.047;
THIGH_OFFSET_Y = 0.08505;
WIDTH_Y = HIP_OFFSET_Y + THIGH_OFFSET_Y;  % = 0.13205 m
LENGTH_X = 0.183;

p_shoulders_body = [ LENGTH_X, -WIDTH_Y, 0; ... 
                     LENGTH_X,  WIDTH_Y, 0; ... 
                    -LENGTH_X, -WIDTH_Y, 0; ... 
                    -LENGTH_X,  WIDTH_Y, 0]';   

% TROT GAIT: 2 diagonal legs in contact, fast but less stable
gait_trot = struct();
gait_trot.T_cycle = 1.0;
gait_trot.stance_percent = 0.55;
gait_trot.phase_offsets = [0.0, 0.5, 0.5, 0.0];  % FR-RL together, FL-RR together

% WALK GAIT: 3 legs in contact at all times, slower but more stable
% Lateral sequence: FR -> RR -> FL -> RL (each 25% offset)
% With 80% stance, more overlap between legs for stability
gait_walk = struct();
gait_walk.T_cycle = 2.0;           % Fixed cycle for stability
gait_walk.stance_percent = 0.80;   % 80% stance = only 20% swing time per leg
gait_walk.phase_offsets = [0.0, 0.5, 0.25, 0.75];  % Proper 0.25 spacing: FR→RR→FL→RL

current_gait = gait_walk;  % Full walk gait - all 4 legs take turns (stationary test)

% Tuning (paper values)
k_raibert = 0.03;  % Paper Eq. 14: k = 0.03
swing_height = 0.04;  % 4cm - increased clearance for forward walking
cmd_body_height = 0.35;  % Reduced to match typical standing height

% MPC Solver
mpc_freq = 40;
dt = 1.0 / mpc_freq;
N_horizon = 20;  % 20 * 0.025s = 0.5s lookahead
MU = 0.6;

%% ==================== MPC WEIGHTS (TUNE HERE) ====================
% State vector: [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz]

% Standing: Moderate orientation weights, robot is stable on 4 legs
Q_stand = diag([200, 200, 10, ...   % roll, pitch, yaw
                20, 20, 50, ...     % px, py, pz
                10, 10, 0.5, ...    % wx, wy, wz (angular vel)
                3, 3, 5]);          % vx, vy, vz (linear vel)

% Locomotion: HIGH orientation weights - 3-leg support is less stable
% Must aggressively correct tilt when one leg is swinging
Q_loco = diag([375, 375, 25, ...    % roll, pitch, yaw
               30, 40, 40, ...      % px, py, pz
               15, 15, 0.5, ...     % wx, wy, wz
               3, 3, 5]);           % vx, vy, vz

% Force weights - allow asymmetric forces for orientation control
R_leg_xy = 1e-4;  % Small penalty on lateral forces
R_leg_z = 1e-5;   % Very small to allow force redistribution for balance
R = diag(repmat([R_leg_xy, R_leg_xy, R_leg_z], 1, 4));
%% =================================================================

% Nominal force per leg (gravity compensation)
f_nominal = zeros(12, 1);
f_nominal([3, 6, 9, 12]) = MASS * GRAVITY / 4;  % Fz for each leg

FSM_STAND = 0;       
FSM_LOCOMOTION = 1;  
current_fsm_state = FSM_STAND; 

%% ========================================================================
%% 2. SETUP & INITIALIZATION
%% ========================================================================

disp('Setting up paths & LCM...');
person_select = 'Ruben_Linux'; 
setup_paths(person_select);
params = initialize_controller_state();

lc = lcm.lcm.LCM.getSingleton();
agg_state = lcm.lcm.MessageAggregator(); agg_state.setMaxMessages(1);
agg_joy = lcm.lcm.MessageAggregator();   agg_joy.setMaxMessages(1);
lc.subscribe(params.STATE_CHANNEL, agg_state);
lc.subscribe(params.JOYSTICK_CHANNEL, agg_joy);

plan_msg = lcm_msgs.mpc_plan_t();

% A1 robot inertia (from a1.xml trunk inertial)
I_body = diag([0.0159, 0.0378, 0.0457]);
I_body_inv = inv(I_body);
g_vec = [0; 0; -GRAVITY];
g_hat_vec = [zeros(9, 1); g_vec * dt];

is_initialized = false;
gait_timer = 0.0;
current_cmd_pos = zeros(3,1);
current_cmd_yaw = 0.0;
foot_pos_start = zeros(3, 4);
debug_cnt = 0;

% Trajectory generation state tracking
prev_contact_state = ones(4, 1);  % Start in stance
touchdown_positions = zeros(3, 4);
leg_phase_timers = zeros(4, 1);  % Time since last state change per leg
standing_foot_positions = zeros(3, 4);  % Fixed positions for standing mode
standing_positions_locked = false;  % Lock positions after settling
lock_timer = 0;  % Force lock after N seconds if criteria not met  

joy = struct('left_stick_x',0,'left_stick_y',0,'right_stick_x',0,'right_stick_y',0);

disp('Waiting for robot state...');

%% ========================================================================
%% 3. MAIN CONTROL LOOP
%% ========================================================================
while true
    loop_start_time = tic;
    
    % --- A. Read Inputs ---
    state_msg = agg_state.getNextMessage(0); 
    if isempty(state_msg)
        pause(0.001); continue; 
    end
    state = lcm_msgs.unitree_a1_state_t(state_msg.data);
    
    joy_msg = agg_joy.getNextMessage(0);
    if ~isempty(joy_msg)
        joy = lcm_msgs.xbox_command_t(joy_msg.data);
    end

    % --- B. First Run Initialization ---
    if ~is_initialized
        current_cmd_pos = state.position;
        current_cmd_pos(3) = cmd_body_height;
        current_cmd_yaw = state.rpy(3);
        foot_pos_start = reshape(state.p_gc, [3, 4]);
        touchdown_positions = foot_pos_start;  % Initialize touchdown positions
        standing_foot_positions = foot_pos_start;  % Fixed positions for standing
        is_initialized = true;

        fprintf('\n========== MPC INITIALIZED ==========\n');
        fprintf('Initial State:\n');
        fprintf('  Position: [%.3f, %.3f, %.3f] m\n', state.position);
        fprintf('  RPY:      [%.1f, %.1f, %.1f] deg\n', state.rpy * 180/pi);
        fprintf('  Velocity: [%.3f, %.3f, %.3f] m/s\n', state.velocity);
        fprintf('\nCommanded:\n');
        fprintf('  Pos Cmd:  [%.3f, %.3f, %.3f] m\n', current_cmd_pos);
        fprintf('  Yaw Cmd:  %.1f deg\n', current_cmd_yaw * 180/pi);
        fprintf('\nFoot Positions (world):\n');
        for i = 1:4
            leg_names = {'FR', 'FL', 'RR', 'RL'};
            fprintf('  %s: [%.3f, %.3f, %.3f]\n', leg_names{i}, foot_pos_start(:,i));
        end

        % Check if initial state is reasonable
        init_pitch = abs(state.rpy(2) * 180/pi);
        init_roll = abs(state.rpy(1) * 180/pi);
        if init_pitch > 10 || init_roll > 10
            fprintf('\n!!! WARNING: Large initial orientation error !!!\n');
            fprintf('  Roll: %.1f deg, Pitch: %.1f deg\n', init_roll, init_pitch);
            fprintf('  Robot may not be properly initialized!\n');
        end
        fprintf('======================================\n\n');
    end

    % --- C. FSM Logic ---
    % Simple: joystick forward = walk, release = stand
    WALK_SPEED = 0.15;  % m/s - forward walk
    move_req = (joy.left_stick_y < -0.1);  % Joystick pushed forward

    switch current_fsm_state
        case FSM_STAND
            if move_req, current_fsm_state = FSM_LOCOMOTION; end
        case FSM_LOCOMOTION
            time_left = current_gait.T_cycle - gait_timer;
            if ~move_req && (time_left < dt*2)
                current_fsm_state = FSM_STAND;
                gait_timer = 0.0;
            end
    end

    % Fixed forward speed when walking, no lateral movement
    if move_req
        v_des_body = [WALK_SPEED; 0; 0];
    else
        v_des_body = [0; 0; 0];
    end

    yaw = state.rpy(3);
    R_z = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    v_des_world = R_z * v_des_body;
    body_omega_cmd = [0; 0; 0];  % No yaw control for now
    des_yaw_rate = 0;  % No yaw control
    des_pitch = 0;  % No pitch control for now

    % --- D. State Execution ---
    if current_fsm_state == FSM_STAND
        % current_cmd_pos(1) = state.position(1); %modified this to test
        % something

        current_cmd_pos(2) = state.position(2);
        current_cmd_pos(3) = cmd_body_height;
        % current_cmd_yaw = state.rpy(3);  % REMOVED: This caused yaw drift by accepting actual as command

        % Calculate center of support --------- testing bs
        % feet_x = state.p_gc([1, 4, 7, 10]);
        % feet_y = state.p_gc([2, 5, 8, 11]);
        % center_x = mean(feet_x);
        % center_y = mean(feet_y);
        % 
        % current_cmd_pos(1) = center_x;
        % current_cmd_pos(2) = center_y;
        % current_cmd_pos(3) = cmd_body_height;
        % current_cmd_yaw = 0;

        contact_cmd = [1; 1; 1; 1];
    else
        gait_timer = mod(gait_timer + dt, current_gait.T_cycle);
        current_cmd_pos = current_cmd_pos + v_des_world * dt;
        current_cmd_pos(3) = cmd_body_height; 
        current_cmd_yaw = current_cmd_yaw + des_yaw_rate * dt;
    end
    
    body_pos_cmd = current_cmd_pos;
    body_vel_cmd = v_des_world;
    body_rpy_cmd = [0; des_pitch; current_cmd_yaw];  % [roll=0, pitch=joystick, yaw=integrated]

    % --- D2. MPC Weight Scheduling Based on FSM State ---
    if current_fsm_state == FSM_STAND
        Q = Q_stand;
    else
        Q = Q_loco;
    end

    % --- E. Gait Scheduler & Trajectory Generation (Paper Methods) ---
    yaw = state.rpy(3);
    R_yaw = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    foot_pos_cmd_world = zeros(12, 1);
    foot_vel_cmd_world = zeros(12, 1);

    % Trajectory parameters (Hyun et al. 2014)
    stance_delta = 0.02;  % m - vertical modulation for stance (Section III-C)

    if current_fsm_state == FSM_STAND
        % DISABLED LOCK-IN: Always use current foot positions (no locking)
        % This tests if delayed lock-in was causing the pitch issues
        standing_foot_positions = reshape(state.p_gc, [3, 4]);
        foot_pos_cmd_world = reshape(standing_foot_positions, [12, 1]);
    else
        for i = 1:4
            [contact, swing_phase] = get_gait_schedule(gait_timer, current_gait.T_cycle, current_gait.stance_percent, current_gait.phase_offsets(i));
            contact_cmd(i) = contact;

            % Update leg phase timers
            if contact ~= prev_contact_state(i)
                leg_phase_timers(i) = 0;  % Reset timer on state change
                if contact == 1
                    % Transitioning to stance: record touchdown position
                    touchdown_positions(:, i) = state.p_gc(3*i-2 : 3*i);
                end
            else
                leg_phase_timers(i) = leg_phase_timers(i) + dt;
            end

            if contact == 1
                % STANCE: Use equilibrium-point hypothesis (Hyun et al. Section III-C)
                T_stance = current_gait.T_cycle * current_gait.stance_percent;
                stance_phase = min(leg_phase_timers(i) / T_stance, 1.0);

                [p_stance, v_stance] = get_stance_trajectory(...
                    touchdown_positions(:, i), stance_phase, T_stance, stance_delta);

                foot_pos_cmd_world(3*i-2 : 3*i) = p_stance;
                foot_vel_cmd_world(3*i-2 : 3*i) = v_stance;
            else
                % SWING: Use Bézier curve trajectory (Hyun et al. Section III-B)
                p_shoulder = state.position + R_yaw * p_shoulders_body(:, i);

                p_target = get_footstep_target(state, ...
                                              v_des_world, ...
                                              body_omega_cmd, ...
                                              p_shoulder, ...
                                              current_gait.T_cycle * current_gait.stance_percent, ...
                                              k_raibert, ...
                                              cmd_body_height, GRAVITY);

                p_target(3) = touchdown_positions(3, i);  % Use previous touchdown height
                T_swing = current_gait.T_cycle * (1 - current_gait.stance_percent);

                [p_swing, v_swing] = get_swing_trajectory_bezier(...
                    touchdown_positions(:, i), p_target, swing_phase, swing_height, T_swing);

                foot_pos_cmd_world(3*i-2 : 3*i) = p_swing;
                foot_vel_cmd_world(3*i-2 : 3*i) = v_swing;
            end
        end

        % Update previous contact state
        prev_contact_state = contact_cmd;
    end

    % --- F. MPC Solver ---
    x_current = [state.rpy; state.position; state.omega; state.velocity];
    x_size = 12; u_size = 12;
    n_vars = (N_horizon + 1) * (x_size + u_size);
    x_ref_traj = zeros(x_size, N_horizon + 1);
    contact_plan = zeros(4, N_horizon + 1);
    
    for k = 0:N_horizon
        t_pred = k * dt;
        ref_pos = body_pos_cmd + body_vel_cmd * t_pred;
        ref_rpy = body_rpy_cmd + body_omega_cmd * t_pred;
        x_ref_traj(:, k+1) = [ref_rpy; ref_pos; body_omega_cmd; body_vel_cmd];
        
        if current_fsm_state == FSM_STAND
             contact_plan(:, k+1) = [1;1;1;1];
        else
             t_future = mod(gait_timer + t_pred, current_gait.T_cycle);
             for leg = 1:4
                 [c, ~] = get_gait_schedule(t_future, current_gait.T_cycle, current_gait.stance_percent, current_gait.phase_offsets(leg));
                 contact_plan(leg, k+1) = c;
             end
        end
    end
    
    H = sparse(n_vars, n_vars); f_vec = sparse(n_vars, 1);
    % Extra rows for total force constraints (one per timestep)
    n_eq_rows = n_vars + (N_horizon + 1);
    A_eq = sparse(n_eq_rows, n_vars); b_eq = sparse(n_eq_rows, 1);
    A_ineq = sparse((N_horizon+1)*20, n_vars); b_ineq = sparse((N_horizon+1)*20, 1);

    x_idx = @(k) (k-1)*x_size + (1:x_size);
    u_idx = @(k) (N_horizon+1)*x_size + (k-1)*u_size + (1:u_size);
    force_eq_idx = @(k) n_vars + k;  % Row index for total force constraint
    ineq_row = 1;
    
    for k = 1:(N_horizon + 1)
        H(x_idx(k), x_idx(k)) = Q; H(u_idx(k), u_idx(k)) = R;
        f_vec(x_idx(k)) = -Q * x_ref_traj(:, k);
        f_vec(u_idx(k)) = -R * f_nominal;  % Penalize deviation from nominal, not zero
        
        ref_yaw_k = x_ref_traj(3, k); 
        ref_pos_k = x_ref_traj(4:6, k); 
        Ak = build_Ak(ref_yaw_k, dt);
        p_feet_mat = reshape(foot_pos_cmd_world, [3, 4]); 
        Bk = build_Bk(MASS, I_body_inv, ref_yaw_k, ref_pos_k, p_feet_mat, dt, contact_plan(:, k));
        
        if k == 1 
             A_eq(x_idx(k), x_idx(k)) = eye(12); A_eq(x_idx(k), u_idx(k)) = -Bk;
             b_eq(x_idx(k)) = Ak * x_current + g_hat_vec;
        else 
             A_eq(x_idx(k), x_idx(k-1)) = -Ak; A_eq(x_idx(k), x_idx(k)) = eye(12);
             A_eq(x_idx(k), u_idx(k)) = -Bk; b_eq(x_idx(k)) = g_hat_vec;
        end
        
        for leg = 1:4
             idx_leg = u_idx(k); idx_leg = idx_leg(3*leg-2 : 3*leg);
             if contact_plan(leg, k) == 0
                 row_range = ineq_row : ineq_row+4;
                 A_ineq(row_range, idx_leg(3)) = [1; -1; 0; 0; 0]; b_ineq(row_range) = 0;
             else
                 cone_mat = [1 0 -MU; -1 0 -MU; 0 1 -MU; 0 -1 -MU; 0 0 -1];
                 row_range = ineq_row : ineq_row+4;
                 A_ineq(row_range, idx_leg) = cone_mat; b_ineq(row_range) = [0;0;0;0;0];
             end
             ineq_row = ineq_row + 5;
        end

        % Total vertical force constraint: sum(f_z) = mg for stance legs
        % This prevents excessive force while allowing redistribution
        n_stance = sum(contact_plan(:, k));
        if n_stance > 0
            eq_row = force_eq_idx(k);
            for leg = 1:4
                if contact_plan(leg, k) == 1
                    fz_idx = u_idx(k); fz_idx = fz_idx(3*leg);  % z-component of leg force
                    A_eq(eq_row, fz_idx) = 1;
                end
            end
            b_eq(eq_row) = MASS * GRAVITY;  % Total vertical force = mg
        end
    end
    
    options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
    [sol, ~, flag] = quadprog(H, f_vec, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
    
    if flag == 1
        reaction_force_cmd = sol(u_idx(1));
    else
        reaction_force_cmd = zeros(12,1);
        reaction_force_cmd([3,6,9,12]) = MASS*GRAVITY/4;
        fprintf('\n!!! MPC QP FAILED !!! flag=%d %s\n', flag, qp_status_str(flag));
        fprintf('  State RPY: [%.1f, %.1f, %.1f] deg\n', state.rpy * 180/pi);
        fprintf('  State Pos: [%.3f, %.3f, %.3f] m\n', state.position);
        fprintf('  Using default equal forces: %.1f N per leg\n', MASS*GRAVITY/4);
    end

    plan_msg.timestamp = state.timestamp;
    plan_msg.contact = contact_cmd;
    plan_msg.reaction_force = reaction_force_cmd;
    plan_msg.body_pos_cmd = body_pos_cmd;
    plan_msg.body_rpy_cmd = body_rpy_cmd;
    plan_msg.body_vel_cmd = body_vel_cmd;
    plan_msg.body_omega_cmd = body_omega_cmd;
    plan_msg.foot_pos_cmd = foot_pos_cmd_world;
    try, plan_msg.foot_vel_cmd = foot_vel_cmd_world; catch, end
    
    lc.publish(params.PLAN_CHANNEL, plan_msg);

    elapsed = toc(loop_start_time);
    if elapsed < dt, pause(dt - elapsed); end
    
    % =========================================================================
    % --- STABILITY MONITOR & LOGS ---
    % =========================================================================
    
    rpy_err_deg = (state.rpy - body_rpy_cmd) * (180/pi); 
    pos_err = state.position - body_pos_cmd;
    
    if (abs(rpy_err_deg(1)) > 30 || abs(rpy_err_deg(2)) > 30)
        fprintf('\n!!! CRASH DETECTED !!!\n');
        fprintf('Last State:\n');
        fprintf('  Roll Err:  %.2f deg\n', rpy_err_deg(1));
        fprintf('  Pitch Err: %.2f deg\n', rpy_err_deg(2));
        fprintf('  Height:    %.3f m (Cmd: %.3f)\n', state.position(3), body_pos_cmd(3));
        fprintf('--------------------------------\n');
        pause(1); 
    end

    debug_cnt = debug_cnt + 1;
    % Print every 20 loops (~0.5s at 40Hz)
    if mod(debug_cnt, 20) == 0
        % Extract forces per leg
        f_FR = reaction_force_cmd(1:3);
        f_FL = reaction_force_cmd(4:6);
        f_RR = reaction_force_cmd(7:9);
        f_RL = reaction_force_cmd(10:12);
        f_total = f_FR + f_FL + f_RR + f_RL;

        % Calculate force asymmetries
        Fz_front = f_FR(3) + f_FL(3);
        Fz_rear = f_RR(3) + f_RL(3);
        Fz_left = f_FL(3) + f_RL(3);
        Fz_right = f_FR(3) + f_RR(3);

        % Calculate moments about COM (approximate)
        p_feet = reshape(foot_pos_cmd_world, [3, 4]);
        M_total = zeros(3, 1);
        forces = [f_FR, f_FL, f_RR, f_RL];
        for i = 1:4
            r = p_feet(:, i) - state.position;
            M_total = M_total + cross(r, forces(:, i));
        end

        % Gait phase for each leg
        phases = zeros(1, 4);
        swing_flags = zeros(1, 4);
        for i = 1:4
            phases(i) = mod(gait_timer / current_gait.T_cycle + current_gait.phase_offsets(i), 1.0);
            swing_flags(i) = phases(i) >= current_gait.stance_percent;
        end

        fprintf('\n==================== MPC [t=%.2fs] ====================\n', debug_cnt * dt);

        % Summary line
        fprintf('QP:%d | H:%.3fm (err:%+.0fmm) | R:%.1f° P:%.1f° Y:%.1f°\n', ...
            flag, state.position(3), pos_err(3)*1000, ...
            state.rpy(1)*180/pi, state.rpy(2)*180/pi, state.rpy(3)*180/pi);

        % Gait info
        phase_pct = gait_timer / current_gait.T_cycle * 100;
        fprintf('Gait: %.0f%% | Stance:[%d%d%d%d] | Vz:%.3fm/s\n', ...
            phase_pct, ~swing_flags, state.velocity(3));

        % Force summary with asymmetry analysis
        fprintf('\n--- FORCES ---\n');
        fprintf('  FR:%5.1f  FL:%5.1f  |  Front: %5.1f N\n', f_FR(3), f_FL(3), Fz_front);
        fprintf('  RR:%5.1f  RL:%5.1f  |  Rear:  %5.1f N\n', f_RR(3), f_RL(3), Fz_rear);
        fprintf('  ─────────────────────────────────\n');
        fprintf('  Total Fz: %.1f N (need: %.1f N) %s\n', ...
            f_total(3), MASS*GRAVITY, force_status(f_total(3), MASS*GRAVITY));
        fprintf('  F-R diff: %+.1f N (pitch ctrl) | L-R diff: %+.1f N (roll ctrl)\n', ...
            Fz_front - Fz_rear, Fz_left - Fz_right);
        fprintf('  Lateral:  Fx=%.1f Fy=%.1f N\n', f_total(1), f_total(2));

        % Moments
        fprintf('\n--- MOMENTS about COM ---\n');
        fprintf('  Mx(roll): %+6.2f Nm | My(pitch): %+6.2f Nm | Mz(yaw): %+6.2f Nm\n', ...
            M_total(1), M_total(2), M_total(3));

        % Errors with trend indicators
        fprintf('\n--- TRACKING ---\n');
        fprintf('  Pos err: X:%+.0fmm Y:%+.0fmm Z:%+.0fmm\n', ...
            pos_err(1)*1000, pos_err(2)*1000, pos_err(3)*1000);
        fprintf('  Ori err: R:%+.1f° P:%+.1f° Y:%+.1f°\n', rpy_err_deg);
        fprintf('  Body pos cmd: X:%+.1f Y:%+.1f Z:%+.1f\n', body_pos_cmd);
        fprintf('  rpy cmd: R:%+.1f° P:%+.1f° Y:%+.1f°\n', body_rpy_cmd);
        fprintf('  Vel:     [%.3f, %.3f, %.3f] m/s\n', state.velocity);

        % Warnings
        if abs(f_total(3) - MASS*GRAVITY) > 20
            fprintf('\n  ⚠ FORCE IMBALANCE: %.1f N off from gravity!\n', ...
                f_total(3) - MASS*GRAVITY);
        end
        if abs(pos_err(3)) > 0.05
            fprintf('  ⚠ LARGE HEIGHT ERROR: %.0f mm\n', pos_err(3)*1000);
        end
        if abs(rpy_err_deg(1)) > 5 || abs(rpy_err_deg(2)) > 5
            fprintf('  ⚠ LARGE ORIENTATION ERROR!\n');
        end
        if abs(Fz_front - Fz_rear) < 2 && abs(rpy_err_deg(2)) > 2
            fprintf('  ⚠ PITCH ERROR but F-R force diff only %.1f N\n', Fz_front - Fz_rear);
        end

        fprintf('=======================================================\n\n');
    end
end

function str = force_status(actual, expected)
    diff_pct = (actual - expected) / expected * 100;
    if abs(diff_pct) < 5
        str = '✓';
    elseif diff_pct > 0
        str = sprintf('↑%.0f%%', diff_pct);
    else
        str = sprintf('↓%.0f%%', abs(diff_pct));
    end
end

function str = qp_status_str(flag)
    switch flag
        case 1, str = '(optimal)';
        case 0, str = '(max iter)';
        case -2, str = '(infeasible)';
        case -3, str = '(unbounded)';
        otherwise, str = sprintf('(unknown: %d)', flag);
    end
end

