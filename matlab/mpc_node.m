clc;
clear;
clear functions;
format compact;

addpath('utils/');
addpath('controllers/');

disp('--- MPC Controller Node (with Gait Scheduler) ---');

%% 1. Setup
disp('Setting up paths...');
person_select = 'David'; 
setup_paths(person_select);

disp('Initializing controller state (for params)...');
params = initialize_controller_state();

disp('Setting up LCM...');
lc = lcm.lcm.LCM.getSingleton();

agg_state = lcm.lcm.MessageAggregator();
agg_state.setMaxMessages(1);
lc.subscribe(params.STATE_CHANNEL, agg_state);

agg_joy = lcm.lcm.MessageAggregator();
agg_joy.setMaxMessages(1);
lc.subscribe(params.JOYSTICK_CHANNEL, agg_joy);

disp(['Listening for state on: ' params.STATE_CHANNEL]);
disp(['Listening for joystick on: ' params.JOYSTICK_CHANNEL]);
disp(['Publishing plan on: ' params.PLAN_CHANNEL]);

%% 2. Setup MPC
mpc_freq = 40; % 40 Hz
dt = 1.0 / mpc_freq;

plan_msg = lcm_msgs.mpc_plan_t();
MASS = 9.0; 
GRAVITY = 9.81;

% --- Gait Scheduler Parameters ---
T_cycle = 0.5; % Gait cycle time (seconds)
stance_percent = 0.5; % 50% stance, 50% swing
T_stance = T_cycle * stance_percent;
swing_height = 0.08; % 8cm swing height
phase_offsets = [0.0, 0.5, 0.5, 0.0]; % FR, FL, RR, RL

% --- Step Planner Parameters ---
k_raibert = 0.03; 
p_shoulders_body = [ 0.183, -0.047, 0; ... % FR
                     0.183,  0.047, 0; ... % FL
                    -0.183, -0.047, 0; ... % RR
                    -0.183,  0.047, 0]';  % (3x4)

% --- MPC Horizon and Cost Parameters (STEP 1) ---
N_horizon = 10; % MPC planning horizon (the 'm' in Eq 10)
x_size = 12;    % State size [Theta, p, omega, p_dot]
u_size = 12;    % Control size [f_1, f_2, f_3, f_4]
MU = 0.7;       % Friction coefficient (for Eq 11)

% Cost matrices (Eq 10)
Q = diag([ ...
    5.0, 5.0, 1.0, ...   % Theta (RPY)
    10.0, 10.0, 20.0, ... % Position (p) - more penalty on Z
    0.1, 0.1, 0.1, ...   % Omega (Angular Vel)
    1.0, 1.0, 1.0 ...    % Velocity (p_dot)
]);

R_leg = 0.0001; 
R = diag(repmat([R_leg, R_leg, R_leg], 1, 4));

% --- Dynamics Helper Variables ---
g_vec = [0; 0; -GRAVITY]; 
g_hat_vec = [zeros(9, 1); g_vec * dt];

% Body Inertia (from Unitree A1 URDF, simplified)
I_body = diag([0.025, 0.064, 0.080]);
I_body_inv = inv(I_body);

%% 3. Main MPC Loop
disp('Waiting for first state message...');
is_initialized = false;
initial_pos = zeros(3,1);
initial_rpy = zeros(3,1);
joy_state = params.joy_state; 
gait_timer = 0.0;
foot_pos_start = zeros(3, 4); % (3x4)

while true
    loop_start_time = tic;
    
    % --- a. Get State and Joystick ---
    state_msg = agg_state.getNextMessage(0); 
    if isempty(state_msg)
        elapsed_time = toc(loop_start_time);
        time_to_wait = dt - elapsed_time;
        if time_to_wait > 0, pause(time_to_wait); end
        continue;
    end
    state = lcm_msgs.unitree_a1_state_t(state_msg.data);
    
    joy_msg = agg_joy.getNextMessage(0);
    if ~isempty(joy_msg)
        joy_lcm = lcm_msgs.xbox_command_t(joy_msg.data);
        joy_state.left_stick_x = joy_lcm.left_stick_x;
        joy_state.left_stick_y = joy_lcm.left_stick_y;
        joy_state.right_stick_x = joy_lcm.right_stick_x;
        joy_state.right_stick_y = joy_lcm.right_stick_y;
    end

    % --- b. Latch initial state ---
    if ~is_initialized
        initial_pos = state.position;
        initial_rpy = state.rpy;
        foot_pos_start = reshape(state.p_gc, [3, 4]);
        is_initialized = true;
        disp('MPC Initialized. Starting Gait Scheduler.');
    end
    
    % --- c. Update Gait Timer ---
    gait_timer = gait_timer + dt;
    if gait_timer > T_cycle
        gait_timer = mod(gait_timer, T_cycle);
    end

    % --- d. Get User Commands ---
    % Left stick controls velocity in the *body frame*
    v_des_body_x = -joy_state.left_stick_y * 0.5; % Forward/Back
    v_des_body_y = -joy_state.left_stick_x * 0.3; % Strafe Left/Right
    v_des_body = [v_des_body_x; v_des_body_y; 0];

    % Right stick controls orientation
    % X-axis controls yaw *rate* (steering)
    des_yaw_rate = -joy_state.right_stick_x * 0.8; % Turn Left/Right
    % Y-axis controls pitch *angle*
    des_pitch_angle = -joy_state.right_stick_y * 0.4; % Look Up/Down
    % We always want to keep roll angle at 0
    des_roll_angle = 0.0;
    
    % --- Convert body-frame velocity to world-frame velocity ---
    % This is critical for the MPC and the footstep planner
    current_yaw = state.rpy(3);
    Rz_yaw = [cos(current_yaw), -sin(current_yaw), 0;
              sin(current_yaw),  cos(current_yaw), 0;
                     0,         0, 1];
    
    v_des_world = Rz_yaw * v_des_body;
    
    % --- e. Run Scheduler and Planner ---
    contact_cmd = zeros(4, 1);
    foot_pos_cmd_world = zeros(12, 1);
    foot_vel_cmd_world = zeros(12, 1);
    
    % R_yaw = eul2rotm([initial_rpy(3), 0, 0], 'XYZ');
    
    % FIX: Define R_yaw manually to avoid toolbox dependency
    yaw = initial_rpy(3);
    R_yaw = [cos(yaw), -sin(yaw), 0;
             sin(yaw),  cos(yaw), 0;
                    0,         0, 1];
    
    for i = 1:4 % For each leg
        % 1. GAIT SCHEDULER
        [contact, swing_phase] = get_gait_schedule(gait_timer, T_cycle, stance_percent, phase_offsets(i));
        contact_cmd(i) = contact;
        
        if contact == 1 % STANCE
            % Stance leg: command it to stay at its current position
            current_foot_pos = state.p_gc(3*i-2 : 3*i);
            foot_pos_start(:, i) = current_foot_pos;
            foot_pos_cmd_world(3*i-2 : 3*i) = current_foot_pos;
        else % SWING
            % 2. STEP PLANNER
            p_shoulder_world = state.position + R_yaw * p_shoulders_body(:, i);
            p_target = get_footstep_target(state, v_des_world, p_shoulder_world, T_stance, k_raibert);
            
            % 3. TRAJECTORY GENERATOR
            T_swing = T_cycle * (1.0 - stance_percent);
            [p_swing, v_swing] = get_swing_trajectory(swing_phase, foot_pos_start(:, i), p_target, swing_height, T_swing);
            
            foot_pos_cmd_world(3*i-2 : 3*i) = p_swing;
            foot_vel_cmd_world(3*i-2 : 3*i) = v_swing;
        end
    end
    
    % --- g. Body Commands ---
    % (We need these desired commands *before* the MPC)
    
    % Desired position: constant height above initial
    body_pos_cmd = initial_pos + [0; 0; 0.1];
    
    % Desired RPY angles: relative to initial, with joystick pitch
    body_rpy_cmd = initial_rpy + [des_roll_angle; des_pitch_angle; 0];
    
    % Desired linear velocity: from our joystick calculation
    body_vel_cmd = v_des_world;
    
    % Desired angular velocity: THIS IS THE KEY FIX
    % We command 0 roll/pitch *rate* (to hold the angle)
    % and the desired yaw *rate* (for steering)
    body_omega_cmd = [0; 0; des_yaw_rate];
    
    % --- f. Build and Solve MPC ---

    % AGGRESSIVE FIX: Force MATLAB to re-read all function files
    % from disk on every loop iteration.
    clear functions; 

    % --- 2.1. Get Current State ---
    x_current = [state.rpy; state.position; state.omega; state.velocity];
    p_feet_world_mat = reshape(state.p_gc, [3, 4]);

    % --- 2.2. Generate Reference Trajectory & Contact Schedule ---
    x_ref_traj = zeros(x_size, N_horizon + 1);
    contact_plan = zeros(4, N_horizon + 1);

    for k = 0:N_horizon % k=0 is current time, k=1...N is future
        k_idx = k + 1; % MATLAB 1-based indexing
        
        current_timer = gait_timer + k * dt;
        if current_timer > T_cycle
            current_timer = mod(current_timer, T_cycle);
        end
        
        % Generate future contact plan
        for i = 1:4
            [contact, ~] = get_gait_schedule(current_timer, T_cycle, stance_percent, phase_offsets(i));
            contact_plan(i, k_idx) = contact;
        end
        
        % Generate future reference state (x_ref)
        
        % Desired linear velocity is constant
        ref_vel = body_vel_cmd; 
        
        % Desired angular velocity is constant
        ref_omega = body_omega_cmd;
        
        % Desired position is integrated forward from k=0
        % body_pos_cmd is the desired pos at k=0
        ref_pos = body_pos_cmd + body_vel_cmd * (k * dt);
        
        % Desired orientation is integrated forward from k=0
        % body_rpy_cmd is the desired RPY at k=0
        % (Simple Euler integration)
        ref_rpy = body_rpy_cmd + body_omega_cmd * (k * dt);
        
        x_ref_traj(:, k_idx) = [ref_rpy; ref_pos; ref_omega; ref_vel];
    end

    % --- 3.1. Pre-allocate QP Matrices ---
    n_states = (N_horizon + 1) * x_size;
    n_controls = (N_horizon + 1) * u_size;
    n_vars = n_states + n_controls;

    H = sparse(n_vars, n_vars);
    f_vec = sparse(n_vars, 1);
    A_eq = sparse(n_vars, n_vars);
    b_eq = sparse(n_vars, 1);

    n_ineq_per_step = 5 * 4; 
    n_ineq = (N_horizon + 1) * n_ineq_per_step;
    A_ineq = sparse(n_ineq, n_vars);
    b_ineq = sparse(n_ineq, 1);

    x_idx = @(k) (k-1)*x_size + (1:x_size); % Indices for x(k)
    u_idx = @(k) n_states + (k-1)*u_size + (1:u_size); % Indices for f(k-1)

    % --- 3.2. Build Cost (H, f_vec) ---
    for k = 1:(N_horizon + 1)
        H(x_idx(k), x_idx(k)) = Q;
        H(u_idx(k), u_idx(k)) = R;
    end
    for k = 1:(N_horizon + 1)
        f_vec(x_idx(k)) = -Q * x_ref_traj(:, k);
    end

    % --- 3.3. Build Dynamics Constraints (A_eq, b_eq) ---
    k_idx = 1; % Corresponds to time k=0
    Ak = build_Ak(x_ref_traj(3, k_idx), dt); % Build Ak using ref yaw
    Bk = build_Bk(MASS, I_body_inv, x_ref_traj(3, k_idx), state.position, p_feet_world_mat, dt, contact_plan(:, k_idx));

    A_eq(x_idx(k_idx), x_idx(k_idx)) = eye(x_size); 
    A_eq(x_idx(k_idx), u_idx(k_idx)) = -Bk;
    b_eq(x_idx(k_idx)) = Ak * x_current + g_hat_vec;

    for k = 1:N_horizon % k is the time step
        k_idx = k + 1; % k_idx is the 1-based index
        
        Ak = build_Ak(x_ref_traj(3, k_idx), dt); 
        Bk = build_Bk(MASS, I_body_inv, x_ref_traj(3, k_idx), state.position, p_feet_world_mat, dt, contact_plan(:, k_idx));

        A_eq(x_idx(k_idx), x_idx(k)) = -Ak;
        A_eq(x_idx(k_idx), u_idx(k_idx)) = -Bk;
        A_eq(x_idx(k_idx), x_idx(k_idx)) = eye(x_size);
        b_eq(x_idx(k_idx)) = g_hat_vec;
    end

    % --- 3.4. Build Friction Cone Constraints (A_ineq, b_ineq) ---
    ineq_row_idx = 1;
    for k = 0:N_horizon % For each time step
        k_idx = k + 1; % 1-based index
        
        for i = 1:4 % For each leg
            f_z_min = 0.0; % Min force
            
            cone_matrix = [ -1,  0, -MU; ...
                             1,  0, -MU; ...
                             0, -1, -MU; ...
                             0,  1, -MU; ...
                             0,  0,  -1 ];
            
            cone_vector = [0; 0; 0; 0; -f_z_min];
            
            if contact_plan(i, k_idx) == 0
                cone_matrix = zeros(5, 3);
                cone_vector = zeros(5, 1);
            end

            leg_control_idx = u_idx(k_idx); 
            leg_control_idx = leg_control_idx( (i-1)*3 + (1:3) ); 
            
            row_range = ineq_row_idx : (ineq_row_idx + 4);
            
            A_ineq(row_range, leg_control_idx) = cone_matrix;
            b_ineq(row_range) = cone_vector;
            
            ineq_row_idx = ineq_row_idx + 5;
        end
    end
    
    % --- 4. Solve the QP ---
    options = optimoptions('quadprog', 'Display', 'none', 'Algorithm', 'interior-point-convex');
    
    Z_opt = [];
    try
        [Z_opt, fval, exitflag] = quadprog(H, f_vec, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
        if exitflag ~= 1
            disp('QP failed to solve or is infeasible.');
            Z_opt = [];
        end
    catch ME
        disp('QP Error:');
        disp(ME.message);
        Z_opt = [];
    end

    if ~isempty(Z_opt)
        % Extract f(0), the forces for the *current* time step
        f_0_opt = Z_opt(u_idx(1));
        reaction_force_cmd = f_0_opt;
    else
        % QP Failed - Use mock forces as a fallback
        disp('QP Failed, using mock forces.');
        reaction_force_cmd = zeros(12, 1);
        current_contacts = contact_plan(:, 1); % Contacts at k=0
        num_stance = sum(current_contacts);

        if num_stance > 0
            f_z = (MASS * GRAVITY) / num_stance;
            if current_contacts(1), reaction_force_cmd(3) = f_z; end
            if current_contacts(2), reaction_force_cmd(6) = f_z; end
            if current_contacts(3), reaction_force_cmd(9) = f_z; end
            if current_contacts(4), reaction_force_cmd(12) = f_z; end
        end
    end
    
    % --- h. Publish the Plan ---
    plan_msg.timestamp = state.timestamp;
    plan_msg.contact = contact_cmd; % Note: This is the *current* contact from scheduler
    plan_msg.reaction_force = reaction_force_cmd;
    plan_msg.body_pos_cmd = body_pos_cmd;
    plan_msg.body_rpy_cmd = body_rpy_cmd;
    plan_msg.body_vel_cmd = body_vel_cmd;
    plan_msg.body_omega_cmd = body_omega_cmd;
    plan_msg.foot_pos_cmd = foot_pos_cmd_world;
    
    lc.publish(params.PLAN_CHANNEL, plan_msg);

    % --- i. Wait for 40 Hz cycle ---
    elapsed_time = toc(loop_start_time);
    time_to_wait = dt - elapsed_time;
    if time_to_wait > 0, pause(time_to_wait); end
end