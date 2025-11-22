function [p_target] = get_footstep_target(state, v_des, omega_des, p_shoulder, T_stance, k_raibert, current_height, gravity)
    % Implements the "Foot Step Planner" (Eq 12-15) with aggressive lateral tuning
    %
    % Inputs:
    %   state          - Robot state struct with .velocity field
    %   v_des          - [3x1] Desired velocity command
    %   omega_des      - [3x1] Desired angular velocity command
    %   p_shoulder     - [3x1] Shoulder position in world frame
    %   T_stance       - Stance duration [s]
    %   k_raibert      - Raibert feedback gain
    %   current_height - Current body height [m]
    %   gravity        - Gravitational acceleration [m/s^2]
    %
    % Output:
    %   p_target       - [3x1] Target footstep position in world frame

    v_curr = state.velocity(1:3);

    %% --- 1. Symmetry Term (Feedforward) ---
    % Use desired velocity for feedforward (more predictable)
    p_symmetry = (T_stance / 2) * v_des;

    %% --- 2. Feedback Term (Raibert Heuristic, Eq 14) ---
    % Boost lateral (Y) gain to reduce sway during trotting
    gain_vector = [k_raibert; k_raibert * 2.0; 0];
    p_feedback = gain_vector .* (v_curr - v_des);

    %% --- 3. Centrifugal Term (Eq 15) ---
    % p_centrifugal = 0.5 * sqrt(h/g) * (v x omega_cmd)
    coeff = 0.5 * sqrt(current_height / gravity);
    p_centrifugal = coeff * cross(v_curr, omega_des);

    %% --- Final Target (Eq 12) ---
    p_target = p_shoulder + p_symmetry + p_feedback + p_centrifugal;
    p_target(3) = 0.0;  % Ground plane
end