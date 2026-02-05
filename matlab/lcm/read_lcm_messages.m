function [state, joy_state, mpc_plan, new_data] = read_lcm_messages(agg_state, agg_joy, agg_plan, last_joy_state, last_mpc_plan)
    % This function performs non-blocking reads on all LCM channels
    % and returns the latest data.
    
    % Initialize outputs

    state = [];

    % new_data seems to be a struct that indicates whether we received either of the 3
    % lcm messages, assume none recieved for initialization

    new_data.state_received = false;
    new_data.joy_received = false;
    new_data.plan_received = false;
    
    joy_state = last_joy_state; % Default to old state (i guess in case a new joy state isn't receieved?)
    mpc_plan = last_mpc_plan;   % Default to old plan (i guess in case a new mpc plan isn't receieved?)

    % --- 1. Check for State Message --- 

    state_msg = agg_state.getNextMessage(0); 
    if ~isempty(state_msg) % if there is a message to read (not empty)
        state = lcm_msgs.unitree_a1_state_t(state_msg.data); % record lcm message into state
        new_data.state_received = true; % set read flag to true
    end

    % --- 2. Check for Joystick Message --- 

    joy_msg = agg_joy.getNextMessage(0);
    if ~isempty(joy_msg)
        joy_lcm = lcm_msgs.xbox_command_t(joy_msg.data);
        joy_state.left_stick_y = joy_lcm.left_stick_y;
        joy_state.right_stick_x = joy_lcm.right_stick_x;
        new_data.joy_received = true;
        
        % fprintf('Joystick Received: L_Y=%.2f, R_X=%.2f\n', joy_lcm.left_stick_y, joy_lcm.right_stick_x); 
    end
    
    % --- 3. Check for MPC Plan Message --- <-- 
    
    plan_msg = agg_plan.getNextMessage(0);
    if ~isempty(plan_msg)
        mpc_plan = lcm_msgs.mpc_plan_t(plan_msg.data);
        new_data.plan_received = true;
        
        % Uncomment this to verify the plan is received
        % fprintf('MPC Plan Received.\n');
    end
end