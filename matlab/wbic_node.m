%% WBIC Controller Node

% Clear everything
clc;
clear;
format compact;

% Add paths to all the relevant folders, this assumes wbic_node.m is being run the right directory (figure it out!)
% This could porbably be adjusted to be more robust and not have the issue I was encountering

addpath('utils/');
addpath('lcm/');
addpath('controllers/');
addpath('gait/');

% Initial CLI Display

disp('--- WBIC Controller ---');

%% 1. Setup 

person_select = 'Ruben_Linux'; %REPLACE WITH WHO IS RUNNING IT, SHOULD JUST BE Ruben_Linux FROM NOW ON
setup_paths(person_select);

params = initialize_controller_state(); % preloads a struct with a bunch of fixed data types and info

[lc, agg_state, agg_joy, agg_plan] = setup_lcm(params); % send the preloaded params into setup_lcm, setups LCM and subscribers/publishers

%% 2. Logging Setup (sets up logs for future reference)

if ~exist('logs', 'dir'), mkdir('logs'); end
logfile = ['logs/wbic_log_' datestr(now, 'yyyy-mm-dd_HH-MM-SS') '.mat'];
data.t = [];
data.rpy = [];
data.rpy_cmd = [];
data.pos = [];
data.pos_cmd = [];
data.omega = [];        % Angular velocity (wx, wy, wz)
data.omega_cmd = [];
data.vel = [];          % Linear velocity (vx, vy, vz)
data.vel_cmd = [];
data.foot_pos = [];
data.foot_pos_cmd = [];
data.foot_force_cmd = [];  % MPC commanded forces (12 components)
data.foot_force_actual = [];  % WBIC computed forces (12 components)

save(logfile, 'data');
fprintf('Logging to: %s\n', logfile);

log_start = tic;
last_save = tic;

% Displays a message right before main loop

disp('Running... (Ctrl+C to stop)');

%% 3. Main Loop

while true

    loop_start = tic; % logs the start time of the current loop

    % reads the incoming lcm messages for the three subscribed channels, 
    % maintains joy stick and mpc plan if no new one is received

    [state, params.joy_state, params.mpc_plan, new_data] = ...
        read_lcm_messages(agg_state, agg_joy, agg_plan, params.joy_state, params.mpc_plan);

    % New state check

    if ~new_data.state_received % if no new state is recieved, pause for the period time, then next loop
        pause(params.dt);
        continue;
    end

    % WBIC Controller Call

    [tau_cmd, contact_state, params, q_j_cmd, q_j_vel_cmd, f_r_actual] = run_wbic_controller(state, params);

    % Publish subsequent control command towards the bot

    publish_control_command(lc, params.control_msg, tau_cmd, contact_state, params, q_j_cmd, q_j_vel_cmd);

    % Log

    data.t(end+1) = toc(log_start);
    data.rpy(:, end+1) = state.rpy;
    data.pos(:, end+1) = state.position;
    data.omega(:, end+1) = state.omega;
    data.vel(:, end+1) = state.velocity;
    data.foot_pos(:, end+1) = state.p_gc;

    data.foot_force_actual(:, end+1) = f_r_actual;  % WBIC computed forces

    if isjava(params.mpc_plan) && ~isempty(params.mpc_plan)
        data.rpy_cmd(:, end+1) = params.mpc_plan.body_rpy_cmd;
        data.pos_cmd(:, end+1) = params.mpc_plan.body_pos_cmd;
        data.omega_cmd(:, end+1) = params.mpc_plan.body_omega_cmd;
        data.vel_cmd(:, end+1) = params.mpc_plan.body_vel_cmd;
        data.foot_pos_cmd(:, end+1) = params.mpc_plan.foot_pos_cmd;
        data.foot_force_cmd(:, end+1) = params.mpc_plan.reaction_force;
    else
        data.rpy_cmd(:, end+1) = [0; 0; state.rpy(3)];
        data.pos_cmd(:, end+1) = [state.position(1:2); 0.35];
        data.omega_cmd(:, end+1) = [0; 0; 0];
        data.vel_cmd(:, end+1) = [0; 0; 0];
        data.foot_pos_cmd(:, end+1) = state.p_gc;
        data.foot_force_cmd(:, end+1) = [0; 0; 12.45*9.81/4; 0; 0; 12.45*9.81/4; 0; 0; 12.45*9.81/4; 0; 0; 12.45*9.81/4];
    end

    % Save every second
    if toc(last_save) > 1
        save(logfile, 'data');
        last_save = tic;
    end

    % Wait unitl a total of 0.001 s have passed (1 loop of 1000 Hz), is this actually happenning? 
    % Could be the case that we are actually running slower than we think, let's log how much a loop takes
    % up unitl this point with a print statement or something, just to confirm we are actually running in time!!

    pause(max(0, params.dt - toc(loop_start)));
    
end
