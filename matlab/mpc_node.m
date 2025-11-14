clc;
clear;
format compact;

addpath('utils/');
addpath('controllers/');

disp('--- MPC Controller Node ---');

%% 1. Setup
disp('Setting up paths...');
person_select = 'David'; 
setup_paths(person_select);

disp('Initializing controller state (for params)...');
params = initialize_controller_state();

disp('Setting up LCM...');
lc = lcm.lcm.LCM.getSingleton();

% --- Subscribe to STATE ---
agg_state = lcm.lcm.MessageAggregator();
agg_state.setMaxMessages(1);
lc.subscribe(params.STATE_CHANNEL, agg_state);

% --- Subscribe to JOYSTICK ---
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

% --- Variables to hold the setpoint ---
is_initialized = false;
initial_pos = zeros(3,1);
initial_rpy = zeros(3,1);
joy_state = params.joy_state; % Use initial state from params

%% 3. Main MPC Loop
while true
    loop_start_time = tic;
    
    % --- a. Get Current Robot State ---
    state_msg = agg_state.getNextMessage(0); 
    if isempty(state_msg)
        %... (wait code) ...
        elapsed_time = toc(loop_start_time);
        time_to_wait = dt - elapsed_time;
        if time_to_wait > 0
            pause(time_to_wait);
        end
        continue;
    end
    state = lcm_msgs.unitree_a1_state_t(state_msg.data);
    
    % --- b. Get Current Joystick State ---
    joy_msg = agg_joy.getNextMessage(0);
    if ~isempty(joy_msg)
        joy_lcm = lcm_msgs.xbox_command_t(joy_msg.data);
        joy_state.left_stick_y = joy_lcm.left_stick_y;
        joy_state.right_stick_x = joy_lcm.right_stick_x;
    end

    % --- c. Latch initial state on first loop ---
    if ~is_initialized
        initial_pos = state.position;
        initial_rpy = state.rpy;
        is_initialized = true;
        disp('MPC Initialized. Holding fixed position.');
    end

    % --- d. Run MPC Logic ---
    
    % 1. Contact state
    contact_cmd = [1; 1; 1; 1];
    
    % 2. Reaction forces
    f_foot = (MASS * GRAVITY) / 4.0;
    reaction_force_cmd = [ 0; 0; f_foot; 0; 0; f_foot; 0; 0; f_foot; 0; 0; f_foot ]; 

    % 3. Body commands
    
    % Position: Hold initial X/Y, hold 0.3m height
    body_pos_cmd = initial_pos; 
    body_pos_cmd(3) = 0.30; % Target height

    % Orientation: Hold initial Yaw, use joystick for Roll/Pitch
    body_rpy_cmd = zeros(3,1);
    body_rpy_cmd(3) = initial_rpy(3); % Hold yaw
    
    % --- ADD JOYSTICK INPUT TO THE PLAN ---
    body_rpy_cmd(2) = body_rpy_cmd(2) + joy_state.left_stick_y * 0.5; % Pitch
    body_rpy_cmd(1) = body_rpy_cmd(1) + joy_state.right_stick_x * 0.3; % Roll
    
    body_vel_cmd = zeros(3,1);
    body_omega_cmd = zeros(3,1);
    foot_pos_cmd = zeros(12,1); 
    
    % --- e. Publish the Plan ---
    plan_msg.timestamp = state.timestamp;
    plan_msg.contact = contact_cmd;
    plan_msg.reaction_force = reaction_force_cmd;
    plan_msg.body_pos_cmd = body_pos_cmd;
    plan_msg.body_rpy_cmd = body_rpy_cmd;
    plan_msg.body_vel_cmd = body_vel_cmd;
    plan_msg.body_omega_cmd = body_omega_cmd;
    plan_msg.foot_pos_cmd = foot_pos_cmd;
    
    lc.publish(params.PLAN_CHANNEL, plan_msg);

    % --- f. Wait for 40 Hz cycle ---
    elapsed_time = toc(loop_start_time);
    time_to_wait = dt - elapsed_time;
    if time_to_wait > 0
        pause(time_to_wait);
    end
end