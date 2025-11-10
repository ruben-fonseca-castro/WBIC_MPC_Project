function [lc, lcm_agg_state, lcm_agg_joystick] = setup_lcm()
    % This function initializes LCM and all message subscriptions.

    STATE_CHANNEL = 'unitree_a1_state'; 
    CONTROL_CHANNEL = 'unitree_a1_control';
    JOYSTICK_CHANNEL = 'XBOX_COMMAND';

    lc = lcm.lcm.LCM.getSingleton();

    % Subscribe to state messages
    lcm_agg_state = lcm.lcm.MessageAggregator();
    lcm_agg_state.setMaxMessages(1); % Only store the latest message
    lc.subscribe(STATE_CHANNEL, lcm_agg_state);

    % Subscribe to joystick messages
    lcm_agg_joystick = lcm.lcm.MessageAggregator();
    lcm_agg_joystick.setMaxMessages(1);
    lc.subscribe(JOYSTICK_CHANNEL, lcm_agg_joystick);
    
    disp(['Listening for state on: ' STATE_CHANNEL]);
    disp(['Listening for joystick on: ' JOYSTICK_CHANNEL]);
    disp(['Publishing control on: ' CONTROL_CHANNEL]);
end