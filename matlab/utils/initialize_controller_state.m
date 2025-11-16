function params = initialize_controller_state()
    params = struct(); 

    % --- Control Timing ---
    params.control_freq = 1000; 
    params.dt = 1.0 / params.control_freq;

    % --- LCM Channels ---
    params.STATE_CHANNEL = 'unitree_a1_state'; 
    params.CONTROL_CHANNEL = 'unitree_a1_control';
    params.JOYSTICK_CHANNEL = 'XBOX_COMMAND';
    params.PLAN_CHANNEL = 'MPC_PLAN'; 
    
    params.control_msg = lcm_msgs.unitree_a1_control_t();

    % --- Desired Pose (Upright "Suggestion") ---
    % This is for Task 3 in the acceleration hierarchy
    params.q_des_base = [0, 0.6, -1.2, ... % FR
                         0, 0.6, -1.2, ... % FL
                         0, 0.6, -1.2, ... % RR
                         0, 0.6, -1.2]';   % RL
                         
    params.q_vel_des = zeros(12, 1);

    % --- PD Gains (for Python Controller) ---
    % These gains are sent to the Python lcm_control_handler
    kp = 5.0;  % Gentle feedback gain
    kd = 0.5;
    params.KP_vec = ones(12, 1) * kp;
    params.KD_vec = ones(12, 1) * kd;

    % --- Initial Joystick State ---
    params.joy_state.left_stick_x = 0.0;
    params.joy_state.left_stick_y = 0.0;
    params.joy_state.right_stick_x = 0.0;
    params.joy_state.right_stick_y = 0.0;
    
    % --- Initial MPC Plan ---
    params.mpc_plan = struct();
    
    % --- Controller State Machine ---
    params.wbc_enabled = true; 
end