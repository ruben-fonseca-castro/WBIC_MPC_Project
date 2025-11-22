%% WBIC Gain Tuning Script
% This script helps tune WBIC gains by visualizing system response
% Run this INSTEAD of wbic_node.m for tuning sessions

clc;
clear;
format compact;

addpath('utils/');
addpath('controllers/');

disp('========================================');
disp('       WBIC Gain Tuning Script');
disp('========================================');

%% Configuration
TEST_DURATION = 10.0;  % seconds
CONTROL_FREQ = 500;    % Hz
DT = 1.0 / CONTROL_FREQ;

%% Setup
disp('Setting up paths & LCM...');
person_select = 'David';
setup_paths(person_select);

lc = lcm.lcm.LCM.getSingleton();

% State subscription
agg_state = lcm.lcm.MessageAggregator();
agg_state.setMaxMessages(1);
lc.subscribe('unitree_a1_state', agg_state);

% MPC plan subscription
agg_plan = lcm.lcm.MessageAggregator();
agg_plan.setMaxMessages(1);
lc.subscribe('unitree_a1_mpc_plan', agg_plan);

disp('Initializing controller state...');
params = initialize_controller_state();

%% Wait for first state
disp('Waiting for robot state...');
state = [];
while isempty(state)
    state_msg = agg_state.getNextMessage(100);
    if ~isempty(state_msg)
        state = lcm_msgs.unitree_a1_state_t(state_msg.data);
    end
end

fprintf('Robot state received!\n');
fprintf('  Position: [%.3f, %.3f, %.3f] m\n', state.position);
fprintf('  RPY: [%.2f, %.2f, %.2f] deg\n', rad2deg(state.rpy));

%% Logging Setup
n_samples = ceil(TEST_DURATION * CONTROL_FREQ);
log = struct();
log.t = zeros(1, n_samples);

% Orientation
log.rpy = zeros(3, n_samples);           % Actual orientation
log.rpy_cmd = zeros(3, n_samples);       % Commanded orientation
log.rpy_error = zeros(3, n_samples);     % Error
log.omega = zeros(3, n_samples);         % Angular velocity

% Position
log.pos = zeros(3, n_samples);           % Actual position
log.pos_cmd = zeros(3, n_samples);       % Commanded position
log.pos_error = zeros(3, n_samples);     % Error
log.vel = zeros(3, n_samples);           % Linear velocity

% Torques
log.tau = zeros(12, n_samples);
log.tau_max = zeros(1, n_samples);

% Forces
log.f_cmd = zeros(12, n_samples);

%% Create Live Figure
fig = figure('Name', 'WBIC Tuning', 'Position', [100, 100, 1400, 900]);

% Orientation tracking
subplot(3, 3, 1);
h_pitch = animatedline('Color', 'b', 'LineWidth', 1.5);
h_pitch_cmd = animatedline('Color', 'r', 'LineStyle', '--');
ylabel('Pitch (deg)');
title('Pitch Tracking');
legend('Actual', 'Cmd', 'Location', 'best');
grid on;

subplot(3, 3, 2);
h_roll = animatedline('Color', 'b', 'LineWidth', 1.5);
h_roll_cmd = animatedline('Color', 'r', 'LineStyle', '--');
ylabel('Roll (deg)');
title('Roll Tracking');
grid on;

subplot(3, 3, 3);
h_yaw = animatedline('Color', 'b', 'LineWidth', 1.5);
h_yaw_cmd = animatedline('Color', 'r', 'LineStyle', '--');
ylabel('Yaw (deg)');
title('Yaw Tracking');
grid on;

% Orientation error
subplot(3, 3, 4);
h_pitch_err = animatedline('Color', 'm', 'LineWidth', 1.5);
ylabel('Pitch Error (deg)');
title('Pitch Error');
grid on;

subplot(3, 3, 5);
h_roll_err = animatedline('Color', 'm', 'LineWidth', 1.5);
ylabel('Roll Error (deg)');
title('Roll Error');
grid on;

subplot(3, 3, 6);
h_height = animatedline('Color', 'b', 'LineWidth', 1.5);
h_height_cmd = animatedline('Color', 'r', 'LineStyle', '--');
ylabel('Height (m)');
title('Height Tracking');
grid on;

% Angular velocity (shows damping)
subplot(3, 3, 7);
h_omega_x = animatedline('Color', 'b', 'LineWidth', 1.5);
ylabel('\omega_x (rad/s)');
xlabel('Time (s)');
title('Roll Rate (damping indicator)');
grid on;

subplot(3, 3, 8);
h_omega_y = animatedline('Color', 'b', 'LineWidth', 1.5);
ylabel('\omega_y (rad/s)');
xlabel('Time (s)');
title('Pitch Rate (damping indicator)');
grid on;

% Max torque
subplot(3, 3, 9);
h_tau_max = animatedline('Color', 'k', 'LineWidth', 1.5);
ylabel('Max |Torque| (Nm)');
xlabel('Time (s)');
title('Peak Joint Torque');
yline(33.5, 'r--', 'Limit');
grid on;

drawnow;

%% Main Control Loop
disp('');
disp('========================================');
disp('     TUNING SESSION STARTED');
disp('========================================');
disp('Publish MPC plans to unitree_a1_mpc_plan');
disp('Or run test_wbic_force_tracking.m');
disp('');

sample_idx = 1;
test_start = tic;
last_plot_time = 0;
PLOT_INTERVAL = 0.05;  % Update plots every 50ms

while sample_idx <= n_samples
    loop_start = tic;

    % --- Read State ---
    state_msg = agg_state.getNextMessage(0);
    if isempty(state_msg)
        pause(0.0001);
        continue;
    end
    state = lcm_msgs.unitree_a1_state_t(state_msg.data);

    % --- Read MPC Plan ---
    plan_msg = agg_plan.getNextMessage(0);
    if ~isempty(plan_msg)
        params.mpc_plan = lcm_msgs.mpc_plan_t(plan_msg.data);
    end

    t = toc(test_start);

    % --- Run WBIC ---
    [tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd, f_r_final] = run_wbic_controller(state, params);

    % --- Publish Commands ---
    publish_control_command(lc, params.control_msg, tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd);

    % --- Log Data ---
    log.t(sample_idx) = t;
    log.rpy(:, sample_idx) = state.rpy;
    log.omega(:, sample_idx) = state.omega;
    log.pos(:, sample_idx) = state.position;
    log.vel(:, sample_idx) = state.velocity;
    log.tau(:, sample_idx) = tau_j;
    log.tau_max(sample_idx) = max(abs(tau_j));

    % Get commands from MPC plan if available
    if isjava(params.mpc_plan) && ~isempty(params.mpc_plan)
        log.rpy_cmd(:, sample_idx) = params.mpc_plan.body_rpy_cmd;
        log.pos_cmd(:, sample_idx) = params.mpc_plan.body_pos_cmd;
        log.f_cmd(:, sample_idx) = params.mpc_plan.reaction_force;
    else
        log.rpy_cmd(:, sample_idx) = [0; 0; state.rpy(3)];  % Default
        log.pos_cmd(:, sample_idx) = [state.position(1:2); 0.35];
    end

    log.rpy_error(:, sample_idx) = state.rpy - log.rpy_cmd(:, sample_idx);
    log.pos_error(:, sample_idx) = state.position - log.pos_cmd(:, sample_idx);

    % --- Update Plots (at reduced rate) ---
    if t - last_plot_time > PLOT_INTERVAL
        % Orientation
        addpoints(h_pitch, t, rad2deg(state.rpy(2)));
        addpoints(h_pitch_cmd, t, rad2deg(log.rpy_cmd(2, sample_idx)));
        addpoints(h_roll, t, rad2deg(state.rpy(1)));
        addpoints(h_roll_cmd, t, rad2deg(log.rpy_cmd(1, sample_idx)));
        addpoints(h_yaw, t, rad2deg(state.rpy(3)));
        addpoints(h_yaw_cmd, t, rad2deg(log.rpy_cmd(3, sample_idx)));

        % Errors
        addpoints(h_pitch_err, t, rad2deg(log.rpy_error(2, sample_idx)));
        addpoints(h_roll_err, t, rad2deg(log.rpy_error(1, sample_idx)));

        % Height
        addpoints(h_height, t, state.position(3));
        addpoints(h_height_cmd, t, log.pos_cmd(3, sample_idx));

        % Angular velocity
        addpoints(h_omega_x, t, state.omega(1));
        addpoints(h_omega_y, t, state.omega(2));

        % Torque
        addpoints(h_tau_max, t, max(abs(tau_j)));

        drawnow limitrate;
        last_plot_time = t;
    end

    % --- Print Status ---
    if mod(sample_idx, 500) == 0
        fprintf('[t=%.1fs] Pitch=%.2f° Roll=%.2f° H=%.3fm | err_pitch=%.2f° err_roll=%.2f° | tau_max=%.1fNm\n', ...
            t, rad2deg(state.rpy(2)), rad2deg(state.rpy(1)), state.position(3), ...
            rad2deg(log.rpy_error(2, sample_idx)), rad2deg(log.rpy_error(1, sample_idx)), ...
            max(abs(tau_j)));
    end

    sample_idx = sample_idx + 1;

    % --- Timing ---
    elapsed = toc(loop_start);
    if elapsed < DT
        pause(DT - elapsed);
    end
end

%% Final Analysis Plots
disp('');
disp('========================================');
disp('     TUNING SESSION COMPLETE');
disp('========================================');

% Trim logs to actual samples
actual_samples = sample_idx - 1;
log.t = log.t(1:actual_samples);
log.rpy = log.rpy(:, 1:actual_samples);
log.rpy_cmd = log.rpy_cmd(:, 1:actual_samples);
log.rpy_error = log.rpy_error(:, 1:actual_samples);
log.omega = log.omega(:, 1:actual_samples);
log.pos = log.pos(:, 1:actual_samples);
log.pos_cmd = log.pos_cmd(:, 1:actual_samples);
log.tau_max = log.tau_max(1:actual_samples);

% Create summary figure
figure('Name', 'WBIC Tuning Summary', 'Position', [150, 150, 1200, 800]);

% Pitch analysis
subplot(2, 3, 1);
plot(log.t, rad2deg(log.rpy(2,:)), 'b', 'LineWidth', 1.5); hold on;
plot(log.t, rad2deg(log.rpy_cmd(2,:)), 'r--', 'LineWidth', 1);
ylabel('Pitch (deg)');
xlabel('Time (s)');
title('Pitch Tracking');
legend('Actual', 'Command');
grid on;

subplot(2, 3, 4);
plot(log.t, rad2deg(log.rpy_error(2,:)), 'm', 'LineWidth', 1.5);
ylabel('Pitch Error (deg)');
xlabel('Time (s)');
title(sprintf('Pitch Error (RMS=%.3f°)', rms(rad2deg(log.rpy_error(2,:)))));
grid on;

% Roll analysis
subplot(2, 3, 2);
plot(log.t, rad2deg(log.rpy(1,:)), 'b', 'LineWidth', 1.5); hold on;
plot(log.t, rad2deg(log.rpy_cmd(1,:)), 'r--', 'LineWidth', 1);
ylabel('Roll (deg)');
xlabel('Time (s)');
title('Roll Tracking');
legend('Actual', 'Command');
grid on;

subplot(2, 3, 5);
plot(log.t, rad2deg(log.rpy_error(1,:)), 'm', 'LineWidth', 1.5);
ylabel('Roll Error (deg)');
xlabel('Time (s)');
title(sprintf('Roll Error (RMS=%.3f°)', rms(rad2deg(log.rpy_error(1,:)))));
grid on;

% Angular velocity (damping indicator)
subplot(2, 3, 3);
plot(log.t, log.omega(2,:), 'b', 'LineWidth', 1.5); hold on;
plot(log.t, log.omega(1,:), 'g', 'LineWidth', 1.5);
ylabel('\omega (rad/s)');
xlabel('Time (s)');
title('Angular Velocity');
legend('\omega_y (pitch rate)', '\omega_x (roll rate)');
grid on;

% Torque usage
subplot(2, 3, 6);
plot(log.t, log.tau_max, 'k', 'LineWidth', 1.5);
yline(33.5, 'r--', 'Torque Limit', 'LineWidth', 1.5);
ylabel('Max |Torque| (Nm)');
xlabel('Time (s)');
title(sprintf('Peak Torque (max=%.1f Nm)', max(log.tau_max)));
grid on;

%% Print Summary Statistics
fprintf('\n--- PERFORMANCE SUMMARY ---\n');
fprintf('Pitch:\n');
fprintf('  RMS Error: %.3f deg\n', rms(rad2deg(log.rpy_error(2,:))));
fprintf('  Max Error: %.3f deg\n', max(abs(rad2deg(log.rpy_error(2,:)))));
fprintf('  Peak Rate: %.3f rad/s\n', max(abs(log.omega(2,:))));

fprintf('Roll:\n');
fprintf('  RMS Error: %.3f deg\n', rms(rad2deg(log.rpy_error(1,:))));
fprintf('  Max Error: %.3f deg\n', max(abs(rad2deg(log.rpy_error(1,:)))));
fprintf('  Peak Rate: %.3f rad/s\n', max(abs(log.omega(1,:))));

fprintf('Torque:\n');
fprintf('  Max Torque: %.1f Nm (limit: 33.5 Nm)\n', max(log.tau_max));
fprintf('  Mean Torque: %.1f Nm\n', mean(log.tau_max));

% Oscillation detection
pitch_rate_crossings = sum(diff(sign(log.omega(2,:))) ~= 0);
roll_rate_crossings = sum(diff(sign(log.omega(1,:))) ~= 0);
fprintf('\nOscillation Indicator (zero-crossings in 10s):\n');
fprintf('  Pitch rate: %d crossings\n', pitch_rate_crossings);
fprintf('  Roll rate: %d crossings\n', roll_rate_crossings);
if pitch_rate_crossings > 50 || roll_rate_crossings > 50
    fprintf('  --> HIGH oscillation detected! Increase kd_ori\n');
elseif pitch_rate_crossings > 20 || roll_rate_crossings > 20
    fprintf('  --> Moderate oscillation. Consider increasing kd_ori slightly\n');
else
    fprintf('  --> Low oscillation. Damping looks good!\n');
end

disp('');
disp('Plots generated. Adjust gains in run_wbic_controller.m and re-run.');
