function [p_swing, v_swing] = get_swing_trajectory(swing_phase, p_start, p_target, swing_height, T_swing)
    % Generates a simple trajectory for the swing foot
    
    % Linear interpolation for X and Y
    p_xy = (1 - swing_phase) * p_start(1:2) + swing_phase * p_target(1:2);
    v_xy = (p_target(1:2) - p_start(1:2)) / T_swing;
    
    % Sinusoidal half-wave for Z
    p_z = swing_height * sin(swing_phase * pi);
    v_z = (swing_height * pi / T_swing) * cos(swing_phase * pi);
    
    p_swing = [p_xy; p_z];
    v_swing = [v_xy; v_z];
end