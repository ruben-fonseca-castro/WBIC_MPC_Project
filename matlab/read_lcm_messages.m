function [state, joy_state, new_data] = read_lcm_messages(lcm_agg_state, lcm_agg_joystick, old_joy_state)
    % This function performs non-blocking reads on all LCM channels
    % and returns the latest data.
    
    % Initialize outputs
    state = [];
    new_data.state_received = false;
    new_data.joy_received = false;
    joy_state = old_joy_state; % Default to old state

    % --- 1. Check for Joystick Message ---
    joy_msg = lcm_agg_joystick.getNextMessage(0);
    if ~isempty(joy_msg)
        joy_lcm = lcm_msgs.xbox_command_t(joy_msg.data);
        joy_state.left_stick_y = joy_lcm.left_stick_y;
        joy_state.right_stick_x = joy_lcm.right_stick_x;
        new_data.joy_received = true;
        
        % Print the joystick values
        fprintf('Joystick Received: L_Y=%.2f, R_X=%.2f\n', joy_lcm.left_stick_y, joy_lcm.right_stick_x);
    end
    
    % --- 2. Check for State Message ---
    state_msg = lcm_agg_state.getNextMessage(0); 
    if ~isempty(state_msg)
        % De-serialize the Java message into a MATLAB struct
        state = lcm_msgs.unitree_a1_state_t(state_msg.data);
        new_data.state_received = true;
    end
end