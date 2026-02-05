clc;
clear;
format compact;

addpath('utils/');
addpath('lcm/');
addpath('controllers/');
addpath('gait/');

disp('--- WBIC Controller ---');

%% 1. Setup
person_select = 'Ruben_Linux';
setup_paths(person_select);
params = initialize_controller_state();
[lc, agg_state, agg_joy, agg_plan] = setup_lcm(params);

%% 2. Logging Setup
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

disp('Running... (Ctrl+C to stop)');

%% 3. Main Loop
while true
    loop_start = tic;

    [state, params.joy_state, params.mpc_plan, new_data] = ...
        read_lcm_messages(agg_state, agg_joy, agg_plan, params.joy_state, params.mpc_plan);

    if ~new_data.state_received
        pause(params.dt);
        continue;
    end

    [tau_cmd, contact_state, params, q_j_cmd, q_j_vel_cmd, f_r_actual] = run_wbic_controller(state, params);
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

    % Wait
    pause(max(0, params.dt - toc(loop_start)));
end
