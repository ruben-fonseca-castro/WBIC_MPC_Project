function [p_stance, v_stance] = get_stance_trajectory(p_touchdown, stance_phase, T_stance, delta_amplitude)
    % Implements Equilibrium-Point Hypothesis for stance trajectory (Hyun et al. 2014, Section III-C)
    % Creates virtual compliance during stance phase to reduce impact forces and improve force tracking
    %
    % Key insight: Instead of holding stance foot rigidly at touch-down position,
    % allow controlled vertical motion following a sinusoidal pattern. This provides:
    %   - Smoother force distribution during stance
    %   - Reduced impact forces at touch-down
    %   - Better impedance control performance
    %   - Improved terrain adaptation
    %
    % Inputs:
    %   p_touchdown     - [3x1] Foot position at touch-down event
    %   stance_phase    - Normalized stance phase ∈ [0, 1]
    %   T_stance        - Stance phase duration (for velocity computation)
    %   delta_amplitude - Vertical deviation amplitude (typically 0.02 m = 2 cm)
    %
    % Outputs:
    %   p_stance - [3x1] Desired stance foot position with modulation
    %   v_stance - [3x1] Desired stance foot velocity

    % Sinusoidal deviation: r(t) = r_td + δ·sin(π·t/T_st)
    % - Starts at touch-down position (sin(0) = 0)
    % - Peaks at mid-stance (sin(π/2) = 1)
    % - Returns to touch-down at lift-off (sin(π) = 0)

    % Vertical position modulation
    z_offset = delta_amplitude * sin(pi * stance_phase);

    % Vertical velocity: v_z = d/dt[δ·sin(π·t/T_st)] = δ·(π/T_st)·cos(π·t/T_st)
    vz_offset = delta_amplitude * (pi / T_stance) * cos(pi * stance_phase);

    % Apply modulation (only in Z-axis; XY remain at touch-down location)
    p_stance = p_touchdown;
    p_stance(3) = p_stance(3) + z_offset;

    % Velocity (only Z component non-zero)
    v_stance = zeros(3, 1);
    v_stance(3) = vz_offset;

    % Note: XY components could also be modulated if lateral compliance is desired,
    % but Hyun et al. only modulate Z for vertical compliance during stance.
end
