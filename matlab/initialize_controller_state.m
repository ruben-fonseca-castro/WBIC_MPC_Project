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
    
    % --- Re-usable LCM Message ---
    params.control_msg = lcm_msgs.unitree_a1_control_t();

    % --- Desired Pose ---
    params.q_des_base = [0, 0.9, -1.8, ... % FR
                         0, 0.9, -1.8, ... % FL
                         0, 0.9, -1.8, ... % RR
                         0, 0.9, -1.8]';   % RL
    params.q_vel_des = zeros(12, 1);

    % --- PD Gains ---
    kp = 20.0;
    kd = 0.2;
    params.KP_vec = ones(12, 1) * kp;
    params.KD_vec = ones(12, 1) * kd;

    % --- Initial Joystick State ---
    params.joy_state.left_stick_y = 0.0;
    params.joy_state.right_stick_x = 0.0;

    % Flag to detect the first run
    params.is_initialized = false; 
    
    % This will store our "intermediate step"
    params.q_des_ramping = zeros(12, 1); 
    
    % Max change in angle per loop (rad/loop)
    % 0.002 rad/loop * 1000 loops/s = 2 rad/s joint speed.
    params.max_step_size = 0.002;

end