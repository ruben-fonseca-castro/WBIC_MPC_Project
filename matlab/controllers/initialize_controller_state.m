function params = initialize_controller_state()
    % This function initializes and preloads params struct with a bunch of stuff

    params = struct(); 

    % --- Control Timing ---

    params.control_freq = 1000; % Frequency of 1000 Hz
    params.dt = 1.0 / params.control_freq; % WBIC period of 0.001 s

    % --- LCM Channels ---

    %Set LCM Channel names

    params.STATE_CHANNEL = 'unitree_a1_state';
    params.CONTROL_CHANNEL = 'unitree_a1_control';
    params.JOYSTICK_CHANNEL = 'XBOX_COMMAND';
    params.PLAN_CHANNEL = 'unitree_a1_mpc_plan'; 
    
    params.control_msg = lcm_msgs.unitree_a1_control_t(); % creates an lcm message 

    % --- Desired Pose (Upright "Suggestion") ---
    % This is for Task 3 in the acceleration hierarchy (what is task 3?)

    % seems like upright suggestion is commanding all the legs to be in the same
    % position, seems super hardcoded, and this is something influencing 
    % the controller at all times?? investigate

    params.q_des_base = [0, 0.6, -1.2, ... % FR
                         0, 0.6, -1.2, ... % FL
                         0, 0.6, -1.2, ... % RR
                         0, 0.6, -1.2]';   % RL
                         
    params.q_vel_des = zeros(12, 1); % holding zero joint velocity, again is this something infuencing all the time thru nullspace?

    % --- PD Gains (for Python Controller) --- 
    % These gains are sent to the Python lcm_control_handler 

    % what is the python controller? lcm_controller_handler tf is this

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

    params.mpc_plan = struct(); %mpc plan thus begins as an empty struct
    
    % --- Controller State Machine ---

    params.wbc_enabled = true; 

    % in the end, returns this params struct with all of this info preloaded from this function
end