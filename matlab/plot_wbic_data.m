%% WBIC Data Plotter
clc; clear; format compact;

%% Load log file
log_dir = 'logs';
files = dir(fullfile(log_dir, 'wbic_log_*.mat'));

if isempty(files)
    error('No log files found in %s/', log_dir);
end

fprintf('Available log files:\n');
for i = 1:length(files)
    fprintf('  %d: %s\n', i, files(i).name);
end

if length(files) == 1
    sel = 1;
else
    sel = input('Select file number (or Enter for latest): ');
    if isempty(sel), sel = length(files); end
end

load(fullfile(log_dir, files(sel).name), 'data');
fprintf('Loaded: %s (%.1f s)\n\n', files(sel).name, data.t(end));

legs = {'FR', 'FL', 'RR', 'RL'};
colors = {'b', 'r', 'g', 'm'};

%% Figure 1: Body Orientation Tracking
figure('Name', 'Body Orientation', 'Position', [50, 500, 600, 400]);

rpy_labels = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    subplot(3, 1, i);
    plot(data.t, rad2deg(data.rpy(i,:)), 'b', 'LineWidth', 1.2); hold on;
    plot(data.t, rad2deg(data.rpy_cmd(i,:)), 'r--', 'LineWidth', 1);
    ylabel([rpy_labels{i} ' (deg)']);
    rmse_val = rms(rad2deg(data.rpy(i,:) - data.rpy_cmd(i,:)));
    title(sprintf('%s  |  RMSE: %.2f deg', rpy_labels{i}, rmse_val));
    legend('Actual', 'Cmd', 'Location', 'best');
    grid on;
    if i == 3, xlabel('Time (s)'); end
end

%% Figure 2: Body Position Tracking
figure('Name', 'Body Position', 'Position', [50, 50, 600, 400]);

pos_labels = {'X', 'Y', 'Z'};
for i = 1:3
    subplot(3, 1, i);
    plot(data.t, data.pos(i,:)*1000, 'b', 'LineWidth', 1.2); hold on;
    plot(data.t, data.pos_cmd(i,:)*1000, 'r--', 'LineWidth', 1);
    ylabel([pos_labels{i} ' (mm)']);
    rmse_val = rms((data.pos(i,:) - data.pos_cmd(i,:)) * 1000);
    title(sprintf('%s  |  RMSE: %.1f mm', pos_labels{i}, rmse_val));
    legend('Actual', 'Cmd', 'Location', 'best');
    grid on;
    if i == 3, xlabel('Time (s)'); end
end

%% Figure 2b: Angular Velocity Tracking
if isfield(data, 'omega') && isfield(data, 'omega_cmd')
    figure('Name', 'Angular Velocity', 'Position', [660, 500, 600, 400]);

    omega_labels = {'\omega_x', '\omega_y', '\omega_z'};
    for i = 1:3
        subplot(3, 1, i);
        plot(data.t, rad2deg(data.omega(i,:)), 'b', 'LineWidth', 1.2); hold on;
        plot(data.t, rad2deg(data.omega_cmd(i,:)), 'r--', 'LineWidth', 1);
        ylabel([omega_labels{i} ' (deg/s)']);
        rmse_val = rms(rad2deg(data.omega(i,:) - data.omega_cmd(i,:)));
        title(sprintf('%s  |  RMSE: %.2f deg/s', omega_labels{i}, rmse_val));
        legend('Actual', 'Cmd', 'Location', 'best');
        grid on;
        if i == 3, xlabel('Time (s)'); end
    end
end

%% Figure 2c: Linear Velocity Tracking
if isfield(data, 'vel') && isfield(data, 'vel_cmd')
    figure('Name', 'Linear Velocity', 'Position', [660, 50, 600, 400]);

    vel_labels = {'v_x', 'v_y', 'v_z'};
    for i = 1:3
        subplot(3, 1, i);
        plot(data.t, data.vel(i,:), 'b', 'LineWidth', 1.2); hold on;
        plot(data.t, data.vel_cmd(i,:), 'r--', 'LineWidth', 1);
        ylabel([vel_labels{i} ' (m/s)']);
        rmse_val = rms(data.vel(i,:) - data.vel_cmd(i,:));
        title(sprintf('%s  |  RMSE: %.3f m/s', vel_labels{i}, rmse_val));
        legend('Actual', 'Cmd', 'Location', 'best');
        grid on;
        if i == 3, xlabel('Time (s)'); end
    end
end

%% Figure 3: Foot Position Tracking
if isfield(data, 'foot_pos') && isfield(data, 'foot_pos_cmd')
    figure('Name', 'Foot Position', 'Position', [700, 500, 900, 600]);

    dims = {'X', 'Y', 'Z'};
    for leg = 1:4
        for dim = 1:3
            subplot(3, 4, (dim-1)*4 + leg);
            idx = (leg-1)*3 + dim;
            plot(data.t, data.foot_pos(idx,:)*1000, 'b', 'LineWidth', 1); hold on;
            plot(data.t, data.foot_pos_cmd(idx,:)*1000, 'r--', 'LineWidth', 0.8);
            rmse_val = rms((data.foot_pos(idx,:) - data.foot_pos_cmd(idx,:)) * 1000);
            if dim == 1
                title(sprintf('%s', legs{leg}));
            end
            ylabel(sprintf('%s (mm)', dims{dim}));
            if dim == 3, xlabel('Time (s)'); end
            text(0.98, 0.95, sprintf('RMSE: %.1f', rmse_val), 'Units', 'normalized', ...
                'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'FontSize', 8);
            grid on;
        end
    end
    sgtitle('Foot Position Tracking');
end

%% Figure 4: Foot Force Tracking
if isfield(data, 'foot_force_cmd') && isfield(data, 'foot_force_actual')
    figure('Name', 'Foot Force', 'Position', [700, 50, 900, 600]);

    dims = {'Fx', 'Fy', 'Fz'};
    for leg = 1:4
        for dim = 1:3
            subplot(3, 4, (dim-1)*4 + leg);
            idx = (leg-1)*3 + dim;
            plot(data.t, data.foot_force_actual(idx,:), 'b', 'LineWidth', 1); hold on;
            plot(data.t, data.foot_force_cmd(idx,:), 'r--', 'LineWidth', 0.8);
            rmse_val = rms(data.foot_force_actual(idx,:) - data.foot_force_cmd(idx,:));
            if dim == 1
                title(sprintf('%s', legs{leg}));
            end
            ylabel(sprintf('%s (N)', dims{dim}));
            if dim == 3, xlabel('Time (s)'); end
            text(0.98, 0.95, sprintf('RMSE: %.1f', rmse_val), 'Units', 'normalized', ...
                'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', 'FontSize', 8);
            grid on;
        end
    end
    sgtitle('Foot Force Tracking (Blue=Actual, Red=Cmd)');
end

%% Print Summary
fprintf('========== SUMMARY ==========\n');
fprintf('Duration: %.1f s\n\n', data.t(end));

fprintf('Body Orientation RMSE:\n');
fprintf('  Roll:  %.2f deg\n', rms(rad2deg(data.rpy(1,:) - data.rpy_cmd(1,:))));
fprintf('  Pitch: %.2f deg\n', rms(rad2deg(data.rpy(2,:) - data.rpy_cmd(2,:))));
fprintf('  Yaw:   %.2f deg\n\n', rms(rad2deg(data.rpy(3,:) - data.rpy_cmd(3,:))));

fprintf('Body Position RMSE:\n');
fprintf('  X: %.1f mm\n', rms((data.pos(1,:) - data.pos_cmd(1,:)) * 1000));
fprintf('  Y: %.1f mm\n', rms((data.pos(2,:) - data.pos_cmd(2,:)) * 1000));
fprintf('  Z: %.1f mm\n', rms((data.pos(3,:) - data.pos_cmd(3,:)) * 1000));

if isfield(data, 'foot_pos') && isfield(data, 'foot_pos_cmd')
    fprintf('\nFoot Position RMSE (mm):\n');
    fprintf('       %6s %6s %6s %6s\n', legs{:});
    dims_label = {'X', 'Y', 'Z'};
    for dim = 1:3
        vals = zeros(1, 4);
        for leg = 1:4
            idx = (leg-1)*3 + dim;
            vals(leg) = rms((data.foot_pos(idx,:) - data.foot_pos_cmd(idx,:)) * 1000);
        end
        fprintf('  %s:  %6.1f %6.1f %6.1f %6.1f\n', dims_label{dim}, vals);
    end
end

if isfield(data, 'foot_force_cmd') && isfield(data, 'foot_force_actual')
    fprintf('\nFoot Force RMSE (N):\n');
    fprintf('       %6s %6s %6s %6s\n', legs{:});
    dims_label = {'Fx', 'Fy', 'Fz'};
    for dim = 1:3
        vals = zeros(1, 4);
        for leg = 1:4
            idx = (leg-1)*3 + dim;
            vals(leg) = rms(data.foot_force_actual(idx,:) - data.foot_force_cmd(idx,:));
        end
        fprintf('  %s: %6.1f %6.1f %6.1f %6.1f\n', dims_label{dim}, vals);
    end
end
fprintf('=============================\n');

%% Save figures as PNG
fig_names = {'Body Orientation', 'Body Position', 'Angular Velocity', 'Linear Velocity', 'Foot Position', 'Foot Force'};
for i = 1:length(fig_names)
    fig = findobj('Type', 'figure', 'Name', fig_names{i});
    if ~isempty(fig)
        saveas(fig, fullfile(log_dir, [fig_names{i} '.png']));
        fprintf('Saved: %s.png\n', fig_names{i});
    end
end
fprintf('\nAll figures saved to %s/\n', log_dir);
