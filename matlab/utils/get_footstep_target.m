function [p_target] = get_footstep_target(state, v_des, p_shoulder, T_stance, k_raibert)
    % Implements the "Foot Step Planner" (Eq 12-14)
    % [cite: image_9a0686.png]
    
    v_curr = state.velocity;
    
    % p_symmetry (Raibert Heuristic, Eq 14)
    % This adjusts the landing spot to control speed
    p_symmetry = (T_stance / 2) * v_curr + k_raibert * (v_curr - v_des);
    
    % p_shoulder (Eq 13)
    % This is the neutral "home" landing spot under the shoulder
    % (We just use the p_shoulder passed in)
    
    % p_centrifugal (Eq 15) - We ignore this for simplicity
    p_centrifugal = zeros(3, 1);
    
    % Final target (Eq 12)
    p_target = p_shoulder + p_symmetry + p_centrifugal;
    
    % Force Z (height) to be zero (on the ground)
    p_target(3) = 0.0; 
end