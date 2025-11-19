function [p_target] = get_footstep_target(state, v_des, omega_des, p_shoulder, T_stance, k_raibert, current_height, gravity)
    % Implements the "Foot Step Planner" (Eq 12-15)
    
    % 1. Get current velocity vector (Eq 14 uses vector v)
    v_curr = state.velocity(1:3);
    
    %% --- Vector Raibert Heuristic (Eq 14) ---
    % The paper uses vector notation: p_sym = (T_st/2)*v + k*(v - v_cmd)
    % This stabilizes BOTH forward (X) and lateral (Y) motion.
    
    p_symmetry = (T_stance / 2) * v_curr + k_raibert * (v_curr - v_des);
    
    % Ensure Z component is zero for the heuristic (we plan on the ground)
    p_symmetry(3) = 0.0;
    
    %% --- Centrifugal Term (Eq 15) ---
    % p_centrifugal = 0.5 * sqrt(h/g) * (v x omega_cmd)
    % This leans the robot into turns to prevent falling outward.
    
    coeff = 0.5 * sqrt(current_height / gravity);
    p_centrifugal = coeff * cross(v_curr, omega_des);
    
    % Ensure Z is zero
    p_centrifugal(3) = 0.0;
    
    %% --- Final Target (Eq 12) ---
    % r_cmd = p_shoulder + p_symmetry + p_centrifugal
    p_target = p_shoulder + p_symmetry + p_centrifugal;
    
    % Enforce ground height (assuming ground is at z=0)
    p_target(3) = 0.0; 
end