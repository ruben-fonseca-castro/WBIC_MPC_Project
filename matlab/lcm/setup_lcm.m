function [lc, agg_state, agg_joy, agg_plan] = setup_lcm(params)
    % This function initializes LCM and all message subscriptions.

    % Initialize LCM

    lc = lcm.lcm.LCM.getSingleton();

    % Subscribe to state messages 

    agg_state = lcm.lcm.MessageAggregator();
    agg_state.setMaxMessages(1);

    lc.subscribe(params.STATE_CHANNEL, agg_state);

    % Subscribe to joystick messages 

    agg_joy = lcm.lcm.MessageAggregator();
    agg_joy.setMaxMessages(1);

    lc.subscribe(params.JOYSTICK_CHANNEL, agg_joy);
    
    % Subscribe to MPC Plan messages 

    agg_plan = lcm.lcm.MessageAggregator();
    agg_plan.setMaxMessages(1);

    lc.subscribe(params.PLAN_CHANNEL, agg_plan);

    % Displays that confiri we are now LCM listening on the three channels, and publishing on control
    
    disp(['Listening for state on: ' params.STATE_CHANNEL]);
    disp(['Listening for joystick on: ' params.JOYSTICK_CHANNEL]);
    disp(['Listening for MPC plan on: ' params.PLAN_CHANNEL]);
    disp(['Publishing control on: ' params.CONTROL_CHANNEL]);
end