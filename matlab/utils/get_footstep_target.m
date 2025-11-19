function [p_target] = get_footstep_target(state, v_des, p_shoulder, T_stance, k_raibert)
    % Implements the "Foot Step Planner" (Eq 12-14)
    
    v_curr = state.velocity;
    
    % --- FIX ---
    % The Raibert heuristic (p_symmetry) is for controlling
    % forward/backward speed. We should only apply it to the
    % X-direction. For strafing (Y-direction), we want the
    % foot to target its neutral shoulder position.
    
    % Extract X-components of velocity
    v_curr_x = v_curr(1);
    v_des_x = v_des(1);
    
    % p_symmetry (Raibert Heuristic, Eq 14) - applied in X only
    p_symmetry_x = (T_stance / 2) * v_curr_x + k_raibert * (v_curr_x - v_des_x);
    
    % The symmetry term is a vector in the world's X-direction
    p_symmetry = [p_symmetry_x; 0; 0];
    
    % p_shoulder (Eq 13)
    % (This is the p_shoulder passed in)
    
    % p_centrifugal (Eq 15) - We ignore this for simplicity
    p_centrifugal = zeros(3, 1);
    
    % Final target (Eq 12)
    p_target = p_shoulder + p_symmetry + p_centrifugal;
    
    % Force Z (height) to be zero (on the ground)
    p_target(3) = 0.0; 
end