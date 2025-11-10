function [tau_cmd, contact_state] = run_pd_controller(state, params)
    % This is the "Joint-Space PD Controller" brain.
    % It calculates torques to reach a desired joint configuration.
    
    % Get current state
    q_pos_curr = state.qj_pos;
    q_vel_curr = state.qj_vel;

    % --- Controller Logic ---
    
    % 1. Calculate desired pose based on joystick "leaning"
    q_des_final = params.q_des_base;
    
    % Add joystick input to hip joints (pitch)
    pitch_offset = params.joy_state.left_stick_y * 0.5; % Max 0.5 rad lean
    q_des_final([2, 5, 8, 11]) = params.q_des_base([2, 5, 8, 11]) + pitch_offset;

    % Add joystick input to abduction joints (roll)
    roll_offset = params.joy_state.right_stick_x * 0.3; % Max 0.3 rad roll
    q_des_final(1) = params.q_des_base(1) + roll_offset; % FR
    q_des_final(4) = params.q_des_base(4) - roll_offset; % FL
    q_des_final(7) = params.q_des_base(7) + roll_offset; % RR
    q_des_final(10) = params.q_des_base(10) - roll_offset; % RL

    % 2. Calculate PD torques
    pos_error = q_des_final - q_pos_curr;
    vel_error = params.q_vel_des - q_vel_curr;
    
    tau_ff = zeros(12, 1); 
    tau_pd = params.KP_vec .* pos_error + params.KD_vec .* vel_error;
    
    % --- Final Outputs ---
    tau_cmd = tau_pd + tau_ff;
    
    % For a PD controller, we just assume all feet are in contact
    contact_state = [1; 1; 1; 1];
end