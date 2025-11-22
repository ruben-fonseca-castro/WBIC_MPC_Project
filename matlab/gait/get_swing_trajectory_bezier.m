function [p_foot, v_foot] = get_swing_trajectory_bezier(p_start, p_target, swing_phase, swing_height, T_swing)
    % Implements 12-point Bézier curve swing trajectory (Hyun et al. 2014, Section III-B)
    % This provides C² continuity for smooth acceleration profiles and reduced impact forces
    %
    % Inputs:
    %   p_start      - [3x1] Starting foot position (touch-down point from previous stance)
    %   p_target     - [3x1] Target foot position (from footstep planner)
    %   swing_phase  - Normalized swing phase ∈ [0, 1]
    %   swing_height - Maximum apex height (typically 0.05-0.10 m)
    %   T_swing      - Swing phase duration (for velocity scaling)
    %
    % Outputs:
    %   p_foot - [3x1] Desired foot position at current phase
    %   v_foot - [3x1] Desired foot velocity at current phase

    % Control points layout (12 points, n=11):
    % P0, P1:       Start region - matched to lift-off
    % P2, P3, P4:   Ramp up to apex
    % P5, P6, P7:   Apex region - maximum swing height
    % P8, P9:       Ramp down from apex
    % P10, P11:     End region - matched to touch-down

    n = 11;  % Degree of Bézier curve (12 control points)
    P = zeros(3, 12);

    % Build control points
    for i = 0:11
        alpha = i / 11;  % Normalized position along trajectory

        % XY plane: Linear interpolation from start to target
        P(1:2, i+1) = (1 - alpha) * p_start(1:2) + alpha * p_target(1:2);

        % Z axis: Bell-shaped curve with flat apex
        if i <= 1
            % Start: on ground (lift-off point)
            P(3, i+1) = p_start(3);
        elseif i >= 10
            % End: on ground (touch-down point)
            P(3, i+1) = p_target(3);
        elseif i >= 5 && i <= 7
            % Apex: maintain maximum height (flat top for clearance)
            P(3, i+1) = swing_height;
        elseif i < 5
            % Ramp up: smooth transition to apex
            % Use quadratic easing for smooth acceleration
            beta = (i - 1) / 4;  % Normalize to [0,1] over indices 2-5
            P(3, i+1) = swing_height * (beta^2);  % Quadratic ramp
        else
            % Ramp down: smooth transition from apex
            % Use quadratic easing for smooth deceleration
            beta = (11 - i) / 3;  % Normalize to [1,0] over indices 8-10
            P(3, i+1) = swing_height * (beta^2);  % Quadratic ramp
        end
    end

    % Evaluate Bézier curve at swing_phase using Bernstein polynomials
    % B(u) = Σᵢ Bᵢ,ₙ(u) · Pᵢ where Bᵢ,ₙ(u) = C(n,i) · u^i · (1-u)^(n-i)

    u = swing_phase;
    p_foot = zeros(3, 1);
    v_foot_unnormalized = zeros(3, 1);

    % Position: B(u)
    for i = 0:n
        B_i = nchoosek(n, i) * u^i * (1-u)^(n-i);
        p_foot = p_foot + B_i * P(:, i+1);
    end

    % Velocity: B'(u) = n · Σᵢ Bᵢ,ₙ₋₁(u) · (Pᵢ₊₁ - Pᵢ)
    for i = 0:(n-1)
        B_i_prime = nchoosek(n-1, i) * u^i * (1-u)^(n-1-i);
        v_foot_unnormalized = v_foot_unnormalized + n * B_i_prime * (P(:, i+2) - P(:, i+1));
    end

    % Scale velocity by phase rate: dB/dt = (dB/du) · (du/dt) = (dB/du) / T_swing
    v_foot = v_foot_unnormalized / T_swing;
end
