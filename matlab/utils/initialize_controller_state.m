function params = initialize_controller_state()
    % This function creates a struct 'params' that holds all
    % persistent controller variables and settings.

    params = struct(); % Initialize empty struct

    % --- Control Timing ---
    params.control_freq = 1000; % Hz
    params.dt = 1.0 / params.control_freq;

    % --- LCM Channels ---
    params.STATE_CHANNEL = 'unitree_a1_state'; 
    params.CONTROL_CHANNEL = 'unitree_a1_control';
    params.JOYSTICK_CHANNEL = 'XBOX_COMMAND';
    params.PLAN_CHANNEL = 'MPC_PLAN'; % <-- **THIS WAS MISSING**
    
    % --- Re-usable LCM Message ---
    params.control_msg = lcm_msgs.unitree_a1_control_t();

    % --- Desired Pose (for WBC secondary task) ---
    params.q_des_base = [0, 0.9, -1.8, ... % FR
                         0, 0.9, -1.8, ... % FL
                         0, 0.9, -1.8, ... % RR
                         0, 0.9, -1.8]';   % RL
    params.q_vel_des = zeros(12, 1);

    % --- PD Gains ---
    kp = 20.0;
    kd = 1.0; % <-- NOTE: I returned this to 1.0 for the WBC
    params.KP_vec = ones(12, 1) * kp;
    params.KD_vec = ones(12, 1) * kd;

    % --- Initial Joystick State ---
    params.joy_state.left_stick_y = 0.0;
    params.joy_state.right_stick_x = 0.0;
    
    % --- Initial MPC Plan ---
    params.mpc_plan = struct(); % <-- **THIS WAS MISSING**
    
    % --- Setpoint Ramping ---
    params.is_initialized = false; 
    params.q_des_ramping = zeros(12, 1); 
    params.max_step_size = 0.002; 
    
    % --- Controller State Machine ---
    params.wbc_enabled = true; % <-- Set to true to run WBC
end