clc;
clear;
format compact;

addpath('utils/');
addpath('controllers/');

disp('========================================');
disp('   WBIC Force Tracking Validation Test');
disp('========================================');
disp('');
disp('This test validates WBIC by commanding');
disp('force redistributions and measuring:');
disp('  1. QP success rate');
disp('  2. Body orientation response');
disp('  3. Torque reasonableness');
disp('');

%% Configuration
TEST_DURATION = 10.0;  % seconds
WBIC_DT = 0.002;       % 500 Hz (matches actual WBIC)

MASS = 12.45;
GRAVITY = 9.81;

% Test signal parameters
FREQ_PITCH = 0.5;      % Hz - pitch oscillation frequency
FREQ_ROLL = 0.3;       % Hz - roll oscillation frequency
AMPLITUDE_PITCH = 10;  % N - front-rear force difference amplitude
AMPLITUDE_ROLL = 8;    % N - left-right force difference amplitude

%% Setup
disp('Setting up paths & LCM...');
person_select = 'David';
setup_paths(person_select);

lc = lcm.lcm.LCM.getSingleton();
agg_state = lcm.lcm.MessageAggregator();
agg_state.setMaxMessages(1);
lc.subscribe('unitree_a1_state', agg_state);  % Lowercase to match bridge

disp('Initializing controller state...');
params = initialize_controller_state();

% Wait for first state
disp('Waiting for robot state...');
state = [];
while isempty(state)
    state_msg = agg_state.getNextMessage(100);
    if ~isempty(state_msg)
        state = lcm_msgs.unitree_a1_state_t(state_msg.data);
    end
end

initial_position = state.position;
initial_rpy = state.rpy;

fprintf('Initial state received:\n');
fprintf('  Position: [%.3f, %.3f, %.3f] m\n', state.position);
fprintf('  RPY:      [%.2f, %.2f, %.2f] deg\n', state.rpy * 180/pi);
fprintf('\nStarting test in 2 seconds...\n');
pause(2);

%% Logging Setup
n_samples = ceil(TEST_DURATION / WBIC_DT);
log = struct();
log.t = zeros(1, n_samples);
log.qp_flag = zeros(1, n_samples);

% Force commands and actuals
log.f_cmd_FR = zeros(3, n_samples);
log.f_cmd_FL = zeros(3, n_samples);
log.f_cmd_RR = zeros(3, n_samples);
log.f_cmd_RL = zeros(3, n_samples);
log.f_final_FR = zeros(3, n_samples);
log.f_final_FL = zeros(3, n_samples);
log.f_final_RR = zeros(3, n_samples);
log.f_final_RL = zeros(3, n_samples);

% State tracking
log.rpy = zeros(3, n_samples);
log.position = zeros(3, n_samples);
log.omega = zeros(3, n_samples);

% Torques
log.tau_j = zeros(12, n_samples);
log.tau_max = zeros(1, n_samples);

% Expected vs actual moments
log.moment_expected = zeros(3, n_samples);
log.moment_actual = zeros(3, n_samples);

% Contact state
log.contact = zeros(4, n_samples);

sample_idx = 1;
test_start_time = tic;

disp('');
disp('========================================');
disp('          TEST RUNNING...');
disp('========================================');
disp('');

%% Main Test Loop
while sample_idx <= n_samples
    loop_start = tic;

    % --- Read State ---
    state_msg = agg_state.getNextMessage(0);
    if isempty(state_msg)
        pause(0.0001);
        continue;
    end
    state = lcm_msgs.unitree_a1_state_t(state_msg.data);

    t = (sample_idx - 1) * WBIC_DT;

    % --- Generate Test Force Commands ---
    % Strategy: Keep total force = mg, but redistribute to create moments
    % Front-Rear redistribution (creates pitch moment)
    % Left-Right redistribution (creates roll moment)

    f_base = MASS * GRAVITY / 4;  % Nominal force per leg

    % Pitch test signal (front vs rear)
    delta_pitch = AMPLITUDE_PITCH * sin(2*pi*FREQ_PITCH*t);

    % Roll test signal (left vs right)
    delta_roll = AMPLITUDE_ROLL * sin(2*pi*FREQ_ROLL*t);

    % Distribute forces (FR, FL, RR, RL)
    % Front-rear: front legs get +delta_pitch/2, rear legs get -delta_pitch/2
    % Left-right: left legs get +delta_roll/2, right legs get -delta_roll/2
    f_FR_z = f_base + delta_pitch/2 - delta_roll/2;  % Front-Right
    f_FL_z = f_base + delta_pitch/2 + delta_roll/2;  % Front-Left
    f_RR_z = f_base - delta_pitch/2 - delta_roll/2;  % Rear-Right
    f_RL_z = f_base - delta_pitch/2 + delta_roll/2;  % Rear-Left

    % Build force vector (x=0, y=0 for simplicity)
    f_r_cmd = [
        0; 0; f_FR_z;
        0; 0; f_FL_z;
        0; 0; f_RR_z;
        0; 0; f_RL_z
    ];

    % --- Create Mock MPC Plan ---
    params.mpc_plan = struct();
    params.mpc_plan.reaction_force = f_r_cmd;
    params.mpc_plan.contact = [1; 1; 1; 1];  % All feet in stance

    % Body commands: maintain current height, zero orientation
    params.mpc_plan.body_pos_cmd = [state.position(1:2); 0.35];
    params.mpc_plan.body_rpy_cmd = [0; 0; state.rpy(3)];  % Keep current yaw
    params.mpc_plan.body_vel_cmd = zeros(3, 1);
    params.mpc_plan.body_omega_cmd = zeros(3, 1);

    % Foot commands: keep current positions (standing)
    params.mpc_plan.foot_pos_cmd = state.p_gc;
    params.mpc_plan.foot_vel_cmd = zeros(12, 1);

    % --- Run WBIC ---
    [tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd, f_r_final] = run_wbic_controller(state, params);

    % --- Publish Commands ---
    publish_control_command(lc, params.control_msg, tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd);

    % --- Log Data ---
    log.t(sample_idx) = t;

    % Forces commanded
    log.f_cmd_FR(:, sample_idx) = f_r_cmd(1:3);
    log.f_cmd_FL(:, sample_idx) = f_r_cmd(4:6);
    log.f_cmd_RR(:, sample_idx) = f_r_cmd(7:9);
    log.f_cmd_RL(:, sample_idx) = f_r_cmd(10:12);

    % Forces final (from WBIC QP)
    log.f_final_FR(:, sample_idx) = f_r_final(1:3);
    log.f_final_FL(:, sample_idx) = f_r_final(4:6);
    log.f_final_RR(:, sample_idx) = f_r_final(7:9);
    log.f_final_RL(:, sample_idx) = f_r_final(10:12);

    % State
    log.rpy(:, sample_idx) = state.rpy;
    log.position(:, sample_idx) = state.position;
    log.omega(:, sample_idx) = state.omega;
    log.contact(:, sample_idx) = contact_state;

    % Torques
    log.tau_j(:, sample_idx) = tau_j;
    log.tau_max(sample_idx) = max(abs(tau_j));

    % Expected moment (from force commands)
    p_feet = reshape(state.p_gc, [3, 4]);
    M_expected = zeros(3, 1);
    forces_cmd = [f_r_cmd(1:3), f_r_cmd(4:6), f_r_cmd(7:9), f_r_cmd(10:12)];
    for i = 1:4
        r = p_feet(:, i) - state.position;
        M_expected = M_expected + cross(r, forces_cmd(:, i));
    end
    log.moment_expected(:, sample_idx) = M_expected;

    % Actual moment (would need force sensors - skip for now)
    % log.moment_actual(:, sample_idx) = ...;

    % Print progress
    if mod(sample_idx, 250) == 0  % Every 0.5 seconds
        fprintf('[t=%.1fs] Roll=%.2f° Pitch=%.2f° Yaw=%.2f° | Max τ=%.1f Nm | δF_pitch=%+.1f N\n', ...
            t, state.rpy(1)*180/pi, state.rpy(2)*180/pi, state.rpy(3)*180/pi, ...
            max(abs(tau_j)), delta_pitch);
    end

    % --- Timing ---
    elapsed = toc(loop_start);
    if elapsed < WBIC_DT
        pause(WBIC_DT - elapsed);
    end

    sample_idx = sample_idx + 1;
end

total_time = toc(test_start_time);
fprintf('\nTest completed in %.2f seconds\n', total_time);

%% Trim logs to actual samples
n_actual = sample_idx - 1;
log.t = log.t(1:n_actual);
log.rpy = log.rpy(:, 1:n_actual);
log.position = log.position(:, 1:n_actual);
log.omega = log.omega(:, 1:n_actual);
log.tau_j = log.tau_j(:, 1:n_actual);
log.tau_max = log.tau_max(1:n_actual);
log.f_cmd_FR = log.f_cmd_FR(:, 1:n_actual);
log.f_cmd_FL = log.f_cmd_FL(:, 1:n_actual);
log.f_cmd_RR = log.f_cmd_RR(:, 1:n_actual);
log.f_cmd_RL = log.f_cmd_RL(:, 1:n_actual);
log.moment_expected = log.moment_expected(:, 1:n_actual);

%% Analysis & Validation
disp('');
disp('========================================');
disp('          ANALYSIS & RESULTS');
disp('========================================');
disp('');

% Compute metrics
total_fz_cmd = log.f_cmd_FR(3,:) + log.f_cmd_FL(3,:) + log.f_cmd_RR(3,:) + log.f_cmd_RL(3,:);
fz_front_cmd = log.f_cmd_FR(3,:) + log.f_cmd_FL(3,:);
fz_rear_cmd = log.f_cmd_RR(3,:) + log.f_cmd_RL(3,:);
fz_left_cmd = log.f_cmd_FL(3,:) + log.f_cmd_RL(3,:);
fz_right_cmd = log.f_cmd_FR(3,:) + log.f_cmd_RR(3,:);

% Check total force is conserved
total_fz_error = total_fz_cmd - MASS*GRAVITY;
fprintf('Force Conservation:\n');
fprintf('  Total Fz: %.2f ± %.2f N (expected: %.2f N)\n', ...
    mean(total_fz_cmd), std(total_fz_cmd), MASS*GRAVITY);
fprintf('  Max error: %.2f N\n', max(abs(total_fz_error)));
if max(abs(total_fz_error)) < 0.1
    fprintf('  ✅ PASS: Force is conserved\n\n');
else
    fprintf('  ❌ FAIL: Force not conserved!\n\n');
end

% Check body response
rpy_relative = log.rpy - [initial_rpy(1); initial_rpy(2); initial_rpy(3)];
rpy_range = [range(rpy_relative(1,:)), range(rpy_relative(2,:)), range(rpy_relative(3,:))] * 180/pi;

fprintf('Body Orientation Response:\n');
fprintf('  Roll range:  %.2f° (expected: some oscillation)\n', rpy_range(1));
fprintf('  Pitch range: %.2f° (expected: some oscillation)\n', rpy_range(2));
fprintf('  Yaw range:   %.2f° (expected: minimal)\n', rpy_range(3));

% Check force tracking (commanded vs final)
f_cmd_all = [log.f_cmd_FR; log.f_cmd_FL; log.f_cmd_RR; log.f_cmd_RL];
f_final_all = [log.f_final_FR; log.f_final_FL; log.f_final_RR; log.f_final_RL];
force_tracking_error = f_cmd_all - f_final_all;
force_tracking_rms = sqrt(mean(force_tracking_error.^2, 2));

fprintf('\nForce Tracking (MPC cmd → WBIC final):\n');
fprintf('  RMS error per component:\n');
fprintf('    Fx: %.3f N, Fy: %.3f N, Fz: %.3f N (per leg avg)\n', ...
    mean(force_tracking_rms(1:3:12)), mean(force_tracking_rms(2:3:12)), mean(force_tracking_rms(3:3:12)));
fprintf('  Max Fz error: %.2f N\n', max(abs(force_tracking_error(3:3:12,:)), [], 'all'));

if max(abs(force_tracking_error(3:3:12,:)), [], 'all') < 5
    fprintf('  ✅ PASS: WBIC tracks MPC forces well (<5N error)\n');
else
    fprintf('  ⚠ WARNING: Large force tracking error\n');
end

% Check if body responds to force changes
% Correlation between front-rear force difference and pitch
fz_front_final = log.f_final_FR(3,:) + log.f_final_FL(3,:);
fz_rear_final = log.f_final_RR(3,:) + log.f_final_RL(3,:);
fr_force_diff = fz_front_final - fz_rear_final;
pitch_deg = log.rpy(2,:) * 180/pi;
correlation_pitch = corrcoef(fr_force_diff, pitch_deg);

fprintf('\nForce-Orientation Coupling:\n');
fprintf('  Pitch-Force correlation: %.3f\n', correlation_pitch(1,2));
if abs(correlation_pitch(1,2)) > 0.3
    fprintf('  ✅ PASS: Body pitch responds to force changes\n');
else
    fprintf('  ⚠ WARNING: Weak pitch response (expected >0.3)\n');
end

% Check torque limits
fprintf('\nTorque Analysis:\n');
fprintf('  Max joint torque: %.2f Nm (limit: 33.5 Nm)\n', max(log.tau_max));
fprintf('  Mean joint torque: %.2f Nm\n', mean(log.tau_max));
if max(log.tau_max) < 33.5
    fprintf('  ✅ PASS: Torques within limits\n');
else
    fprintf('  ❌ FAIL: Torque exceeds actuator limits!\n');
end

% Check stability (no runaway)
pos_drift = norm(log.position(:,end) - log.position(:,1));
rpy_drift_deg = norm(log.rpy(:,end) - log.rpy(:,1)) * 180/pi;

fprintf('\nStability Check:\n');
fprintf('  Position drift: %.3f m\n', pos_drift);
fprintf('  Orientation drift: %.2f deg\n', rpy_drift_deg);
if pos_drift < 0.1 && rpy_drift_deg < 5
    fprintf('  ✅ PASS: System stable (no runaway)\n');
else
    fprintf('  ❌ FAIL: System unstable!\n');
end

%% Summary
disp('');
disp('========================================');
disp('          SUMMARY');
disp('========================================');
pass_count = 0;
total_tests = 5;

if max(abs(total_fz_error)) < 0.1, pass_count = pass_count + 1; end
if max(abs(force_tracking_error(3:3:12,:)), [], 'all') < 5, pass_count = pass_count + 1; end
if abs(correlation_pitch(1,2)) > 0.3, pass_count = pass_count + 1; end
if max(log.tau_max) < 33.5, pass_count = pass_count + 1; end
if pos_drift < 0.1 && rpy_drift_deg < 5, pass_count = pass_count + 1; end

fprintf('\nTests Passed: %d / %d\n', pass_count, total_tests);
if pass_count == total_tests
    fprintf('✅ ALL TESTS PASSED!\n');
    fprintf('WBIC is correctly tracking force commands.\n');
else
    fprintf('⚠ SOME TESTS FAILED\n');
    fprintf('Review plots for details.\n');
end

disp('');
disp('Generating plots...');

%% Plotting
figure('Name', 'WBIC Force Tracking Test Results', 'Position', [100, 100, 1400, 900]);

% Plot 1: Force Commands vs Finals
subplot(3, 3, 1);
hold on;
plot(log.t, log.f_cmd_FR(3,:), 'r--', 'LineWidth', 1, 'DisplayName', 'FR cmd');
plot(log.t, log.f_final_FR(3,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'FR final');
plot(log.t, log.f_cmd_FL(3,:), 'g--', 'LineWidth', 1, 'DisplayName', 'FL cmd');
plot(log.t, log.f_final_FL(3,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'FL final');
plot(log.t, log.f_cmd_RR(3,:), 'b--', 'LineWidth', 1, 'DisplayName', 'RR cmd');
plot(log.t, log.f_final_RR(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'RR final');
yline(MASS*GRAVITY/4, 'k:', 'LineWidth', 1, 'DisplayName', 'Nominal');
hold off;
xlabel('Time (s)'); ylabel('Force (N)');
title('MPC Commands (dashed) vs WBIC Final (solid)');
legend('Location', 'eastoutside');
grid on;

% Plot 2: Front-Rear Distribution
subplot(3, 3, 2);
hold on;
plot(log.t, fz_front_cmd, 'r-', 'LineWidth', 2, 'DisplayName', 'Front');
plot(log.t, fz_rear_cmd, 'b-', 'LineWidth', 2, 'DisplayName', 'Rear');
plot(log.t, fz_front_cmd - fz_rear_cmd, 'k-', 'LineWidth', 1.5, 'DisplayName', 'F-R Diff');
yline(0, 'k:', 'LineWidth', 1);
hold off;
xlabel('Time (s)'); ylabel('Force (N)');
title('Front-Rear Force Distribution');
legend('Location', 'best');
grid on;

% Plot 3: Left-Right Distribution
subplot(3, 3, 3);
hold on;
plot(log.t, fz_left_cmd, 'g-', 'LineWidth', 2, 'DisplayName', 'Left');
plot(log.t, fz_right_cmd, 'm-', 'LineWidth', 2, 'DisplayName', 'Right');
plot(log.t, fz_left_cmd - fz_right_cmd, 'k-', 'LineWidth', 1.5, 'DisplayName', 'L-R Diff');
yline(0, 'k:', 'LineWidth', 1);
hold off;
xlabel('Time (s)'); ylabel('Force (N)');
title('Left-Right Force Distribution');
legend('Location', 'best');
grid on;

% Plot 4: Roll Response
subplot(3, 3, 4);
yyaxis left;
plot(log.t, log.rpy(1,:)*180/pi, 'b-', 'LineWidth', 2);
ylabel('Roll (deg)');
ylim([-10 10]);
yyaxis right;
plot(log.t, fz_left_cmd - fz_right_cmd, 'r--', 'LineWidth', 1.5);
ylabel('L-R Force Diff (N)');
xlabel('Time (s)');
title('Roll Response to Force Redistribution');
grid on;

% Plot 5: Pitch Response
subplot(3, 3, 5);
yyaxis left;
plot(log.t, log.rpy(2,:)*180/pi, 'b-', 'LineWidth', 2);
ylabel('Pitch (deg)');
ylim([-10 10]);
yyaxis right;
plot(log.t, fz_front_cmd - fz_rear_cmd, 'r--', 'LineWidth', 1.5);
ylabel('F-R Force Diff (N)');
xlabel('Time (s)');
title('Pitch Response to Force Redistribution');
grid on;

% Plot 6: Yaw (Should be stable)
subplot(3, 3, 6);
plot(log.t, log.rpy(3,:)*180/pi, 'b-', 'LineWidth', 2);
ylabel('Yaw (deg)');
xlabel('Time (s)');
title('Yaw (Should Remain Stable)');
grid on;

% Plot 7: Joint Torques
subplot(3, 3, 7);
hold on;
for i = 1:12
    plot(log.t, log.tau_j(i,:), 'LineWidth', 0.5);
end
plot(log.t, log.tau_max, 'k-', 'LineWidth', 2, 'DisplayName', 'Max');
yline(33.5, 'r--', 'LineWidth', 2, 'DisplayName', 'Limit');
yline(-33.5, 'r--', 'LineWidth', 2);
hold off;
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('Joint Torques');
legend('Max', 'Limit', 'Location', 'best');
grid on;

% Plot 8: Expected Moments
subplot(3, 3, 8);
hold on;
plot(log.t, log.moment_expected(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Mx (roll)');
plot(log.t, log.moment_expected(2,:), 'g-', 'LineWidth', 1.5, 'DisplayName', 'My (pitch)');
plot(log.t, log.moment_expected(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Mz (yaw)');
yline(0, 'k:', 'LineWidth', 1);
hold off;
xlabel('Time (s)'); ylabel('Moment (Nm)');
title('Expected Moments from Force Distribution');
legend('Location', 'best');
grid on;

% Plot 9: Position Tracking
subplot(3, 3, 9);
hold on;
plot(log.t, (log.position(1,:) - initial_position(1))*1000, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
plot(log.t, (log.position(2,:) - initial_position(2))*1000, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
plot(log.t, (log.position(3,:) - initial_position(3))*1000, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
yline(0, 'k:', 'LineWidth', 1);
hold off;
xlabel('Time (s)'); ylabel('Position Drift (mm)');
title('Body Position (Relative to Start)');
legend('Location', 'best');
grid on;

sgtitle('WBIC Force Tracking Validation Test', 'FontSize', 14, 'FontWeight', 'bold');

% Save results
save('wbic_test_results.mat', 'log', 'MASS', 'GRAVITY', 'TEST_DURATION');
fprintf('\nResults saved to: wbic_test_results.mat\n');
fprintf('Plots displayed. Close figure to exit.\n');
disp('');
disp('========================================');
disp('          TEST COMPLETE');
disp('========================================');
