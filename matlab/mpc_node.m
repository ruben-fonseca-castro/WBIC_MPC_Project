clc;
clear;
format compact;

addpath('utils/');

disp('--- MPC Controller ---');

%% 1. Setup
disp('Setting up paths...');
person_select = 'David'; % ('David' or 'Ruben' or 'Pranav')
setup_paths(person_select);

disp('Initializing controller...');
% Use a struct to hold all controller parameters
params = initialize_controller_state();

disp('Setting up LCM...');
lc = lcm.lcm.LCM.getSingleton();

% Subscribe ONLY to the state channel
agg_state = lcm.lcm.MessageAggregator();
agg_state.setMaxMessages(1);
lc.subscribe(params.STATE_CHANNEL, agg_state);

disp(['Listening for state on: ' params.STATE_CHANNEL]);
disp(['Publishing plan on: ' params.PLAN_CHANNEL]);

%% 2. Setup MPC
mpc_freq = 40; % 40 Hz, as per the paper
dt = 1.0 / mpc_freq;

% Create a re-usable plan message
plan_msg = lcm_msgs.mpc_plan_t();

% Get robot mass and gravity
MASS = 9.0; % Approx. mass of A1
GRAVITY = 9.81;

%% 3. Main MPC Loop
while true
    loop_start_time = tic;
    
    % --- a. Get Current Robot State ---
    msg = agg_state.getNextMessage(0); 
    if isempty(msg)
        % Wait for next cycle if no state message
        elapsed_time = toc(loop_start_time);
        time_to_wait = dt - elapsed_time;
        if time_to_wait > 0
            pause(time_to_wait);
        end
        continue;
    end
    state = lcm_msgs.unitree_a1_state_t(msg.data);

    % --- b. Run MPC Logic ---
    % For now, we will create a "dummy" plan just for STANDING.
    
    % 1. Contact state: All 4 feet on the ground
    contact_cmd = [1; 1; 1; 1];
    
    % 2. Reaction forces: Equal distribution of weight
    f_foot = (MASS * GRAVITY) / 4.0;
    reaction_force_cmd = [ 0; 0; f_foot; ... % FR
                           0; 0; f_foot; ... % FL
                           0; 0; f_foot; ... % RR
                           0; 0; f_foot ];  % RL

    % 3. Body/Foot commands: Just hold the current state
    body_pos_cmd = state.position;
    body_rpy_cmd = state.rpy;
    body_vel_cmd = zeros(3,1);
    body_omega_cmd = zeros(3,1);
    foot_pos_cmd = zeros(12,1); 
    
    % --- 4c. Publish the Plan ---
    plan_msg.timestamp = state.timestamp;
    plan_msg.contact = contact_cmd;
    plan_msg.reaction_force = reaction_force_cmd;
    plan_msg.body_pos_cmd = body_pos_cmd;
    plan_msg.body_rpy_cmd = body_rpy_cmd;
    plan_msg.body_vel_cmd = body_vel_cmd;
    plan_msg.body_omega_cmd = body_omega_cmd;
    plan_msg.foot_pos_cmd = foot_pos_cmd;
    
    lc.publish(params.PLAN_CHANNEL, plan_msg);

    % --- 4d. Wait for 40 Hz cycle ---
    elapsed_time = toc(loop_start_time);
    time_to_wait = dt - elapsed_time;
    if time_to_wait > 0
        pause(time_to_wait);
    end
end