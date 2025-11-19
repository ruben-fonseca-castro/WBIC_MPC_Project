clc;
clear;
clear functions; 
format compact;

addpath('utils/');
addpath('controllers/');

disp('--- MPC Controller Node (Stable Wide Trot) ---');

%% ========================================================================
%% 1. CONFIGURATION & PARAMETERS
%% ========================================================================

TEST_TROT_IN_PLACE = true; 

MASS = 12.45; 
GRAVITY = 9.81;

% FIX 1: WIDER STANCE (Add 0.05m width)
% This gives the robot a wider base to stop the side-to-side rocking
HIP_OFFSET_Y = 0.047;
THIGH_OFFSET_Y = 0.085;
WIDTH_Y = HIP_OFFSET_Y + THIGH_OFFSET_Y + 0.08;
LENGTH_X = 0.183;

p_shoulders_body = [ LENGTH_X, -WIDTH_Y, 0; ... 
                     LENGTH_X,  WIDTH_Y, 0; ... 
                    -LENGTH_X, -WIDTH_Y, 0; ... 
                    -LENGTH_X,  WIDTH_Y, 0]';   

% FIX 2: CHANGE TIMING
gait_trot = struct();
gait_trot.T_cycle = 0.5;
gait_trot.stance_percent = 0.55; 
gait_trot.phase_offsets = [0.0, 0.5, 0.5, 0.0]; 

current_gait = gait_trot;

% Tuning
k_raibert = 0.15; % Was 0.08
swing_height = 0.10;    
cmd_body_height = 0.28; 

% MPC Solver
mpc_freq = 40; 
dt = 1.0 / mpc_freq; 
N_horizon = 10;
MU = 0.6; 

Q = diag([20.0, 30.0, 5.0, 10.0, 10.0, 50.0, 0.1, 0.1, 0.1, 1.0, 1.0, 1.0]);
R_leg = 1e-4;
R = diag(repmat([R_leg, R_leg, R_leg], 1, 4));

FSM_STAND = 0;       
FSM_LOCOMOTION = 1;  
current_fsm_state = FSM_STAND; 

%% ========================================================================
%% 2. SETUP & INITIALIZATION
%% ========================================================================

disp('Setting up paths & LCM...');
person_select = 'David'; 
setup_paths(person_select);
params = initialize_controller_state();

lc = lcm.lcm.LCM.getSingleton();
agg_state = lcm.lcm.MessageAggregator(); agg_state.setMaxMessages(1);
agg_joy = lcm.lcm.MessageAggregator();   agg_joy.setMaxMessages(1);
lc.subscribe(params.STATE_CHANNEL, agg_state);
lc.subscribe(params.JOYSTICK_CHANNEL, agg_joy);

plan_msg = lcm_msgs.mpc_plan_t();

I_body = diag([0.025, 0.064, 0.080]);
I_body_inv = inv(I_body);
g_vec = [0; 0; -GRAVITY];
g_hat_vec = [zeros(9, 1); g_vec * dt];

is_initialized = false;
gait_timer = 0.0;
current_cmd_pos = zeros(3,1); 
current_cmd_yaw = 0.0;       
foot_pos_start = zeros(3, 4); 
debug_cnt = 0;  

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
        is_initialized = true;
        disp('MPC Initialized. Starting in FSM_STAND.');
    end

    % --- C. FSM Logic ---
    if TEST_TROT_IN_PLACE
        current_fsm_state = FSM_LOCOMOTION;
        v_des_world = zeros(3, 1);
        body_omega_cmd = zeros(3, 1);
        des_yaw_rate = 0;
    else
        joy_vec = [joy.left_stick_x, joy.left_stick_y, joy.right_stick_x];
        move_req = (norm(joy_vec) > 0.1);
        
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
        
        v_des_body = [-joy.left_stick_y * 0.6; -joy.left_stick_x * 0.3; 0];
        des_yaw_rate = -joy.right_stick_x * 1.0;
        yaw = state.rpy(3);
        R_z = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
        v_des_world = R_z * v_des_body;
        body_omega_cmd = [0; 0; des_yaw_rate];
    end

    % --- D. State Execution ---
    if current_fsm_state == FSM_STAND
        current_cmd_pos(1) = state.position(1);
        current_cmd_pos(2) = state.position(2);
        current_cmd_pos(3) = cmd_body_height;
        current_cmd_yaw = state.rpy(3);
        contact_cmd = [1; 1; 1; 1];
    else
        gait_timer = mod(gait_timer + dt, current_gait.T_cycle);
        current_cmd_pos = current_cmd_pos + v_des_world * dt;
        current_cmd_pos(3) = cmd_body_height; 
        current_cmd_yaw = current_cmd_yaw + des_yaw_rate * dt;
    end
    
    body_pos_cmd = current_cmd_pos;
    body_vel_cmd = v_des_world;
    body_rpy_cmd = [0; 0; current_cmd_yaw];

    % --- E. Gait Scheduler ---
    yaw = state.rpy(3);
    R_yaw = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1];
    foot_pos_cmd_world = zeros(12, 1);
    foot_vel_cmd_world = zeros(12, 1);
    
    if current_fsm_state == FSM_STAND
        foot_pos_cmd_world = state.p_gc; 
    else
        for i = 1:4
            [contact, swing_phase] = get_gait_schedule(gait_timer, current_gait.T_cycle, current_gait.stance_percent, current_gait.phase_offsets(i));
            contact_cmd(i) = contact;
            
            if contact == 1
                foot_pos_start(:, i) = state.p_gc(3*i-2 : 3*i);
                foot_pos_cmd_world(3*i-2 : 3*i) = foot_pos_start(:, i);
                foot_vel_cmd_world(3*i-2 : 3*i) = zeros(3, 1);
            else
                p_shoulder = state.position + R_yaw * p_shoulders_body(:, i);
                
                p_target = get_footstep_target_aggressive(state, ...
                                                          v_des_world, ... 
                                                          body_omega_cmd, ...
                                                          p_shoulder, ...
                                                          current_gait.T_cycle * current_gait.stance_percent, ...
                                                          k_raibert, ...
                                                          cmd_body_height, GRAVITY);
                                                      
                p_target(3) = foot_pos_start(3, i); 
                T_swing = current_gait.T_cycle * (1 - current_gait.stance_percent);
                
                [p_swing, v_swing] = get_swing_traj_cosine(swing_phase, foot_pos_start(:, i), p_target, swing_height, T_swing);
                
                foot_pos_cmd_world(3*i-2 : 3*i) = p_swing;
                foot_vel_cmd_world(3*i-2 : 3*i) = v_swing;
            end
        end
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
    A_eq = sparse(n_vars, n_vars); b_eq = sparse(n_vars, 1);
    A_ineq = sparse((N_horizon+1)*20, n_vars); b_ineq = sparse((N_horizon+1)*20, 1);
    
    x_idx = @(k) (k-1)*x_size + (1:x_size);
    u_idx = @(k) (N_horizon+1)*x_size + (k-1)*u_size + (1:u_size);
    ineq_row = 1;
    
    for k = 1:(N_horizon + 1)
        H(x_idx(k), x_idx(k)) = Q; H(u_idx(k), u_idx(k)) = R;
        f_vec(x_idx(k)) = -Q * x_ref_traj(:, k);
        
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
    end
    
    options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
    [sol, ~, flag] = quadprog(H, f_vec, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
    
    if flag == 1, reaction_force_cmd = sol(u_idx(1));
    else, reaction_force_cmd = zeros(12,1); reaction_force_cmd([3,6,9,12]) = MASS*GRAVITY/4; end

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
    % FIX 3: Print every 17 loops (Prime Number) to see all phases
    if mod(debug_cnt, 17) == 0
        fprintf('-----------------------------------------------------------\n');
        fprintf('T:%.2f | STATE: %d | CONTACT: [%d %d %d %d]\n', ...
            gait_timer, current_fsm_state, contact_cmd);
        fprintf('  STABILITY: RollErr=%.1f deg | PitchErr=%.1f deg\n', ...
            rpy_err_deg(1), rpy_err_deg(2));
        fprintf('  HEIGHT:    Cmd=%.3f | Act=%.3f | Err=%.3f\n', ...
            body_pos_cmd(3), state.position(3), pos_err(3));
        fprintf('  FR_FOOT:   CmdZ=%.3f | ActZ=%.3f | Diff=%.3f\n', ...
            foot_pos_cmd_world(3), state.p_gc(3), foot_pos_cmd_world(3) - state.p_gc(3));
        fprintf('-----------------------------------------------------------\n');
    end
end

% --- LOCAL FUNCTIONS ---
function [p, v] = get_swing_traj_cosine(phase, p_start, p_end, h, T_swing)
    p_xy = (1-phase)*p_start(1:2) + phase*p_end(1:2);
    v_xy = (p_end(1:2) - p_start(1:2)) / T_swing;
    ground_z = (1-phase)*p_start(3) + phase*p_end(3);
    p_z = ground_z + (h/2) * (1 - cos(2*pi*phase));
    v_z = (h * pi / T_swing) * sin(2*pi*phase);
    p = [p_xy; p_z]; v = [v_xy; v_z];
end

function [p_target] = get_footstep_target_aggressive(state, v_des, omega_des, p_shoulder, T_stance, k_raibert, current_height, gravity)
    % 1. Symmetry (Feedforward)
    p_symmetry = (T_stance / 2) * v_des; 
    
    % 2. Feedback (Raibert)
    v_curr = state.velocity(1:3);
    % Boost Lateral Feedback Gain (Y-axis) to stop swaying
    gain_vector = [k_raibert; k_raibert * 2.0; 0]; 
    p_feedback = gain_vector .* (v_curr - v_des);
    
    % 3. Centrifugal
    coeff = 0.5 * sqrt(current_height / gravity);
    p_centrifugal = coeff * cross(v_curr, omega_des);
    
    p_target = p_shoulder + p_symmetry + p_feedback + p_centrifugal;
    p_target(3) = 0.0;
end