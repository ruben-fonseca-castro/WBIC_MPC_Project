function publish_control_command(lc, control_msg, tau_cmd, contact_state, params)
    % This function fills and publishes the final LCM control message.

    control_msg.timestamp = int64(java.lang.System.currentTimeMillis() * 1000);
    control_msg.qj_tau = tau_cmd;   
    control_msg.kp = params.KP_vec; % Send gains (good practice)
    control_msg.kd = params.KD_vec;
    control_msg.contact = contact_state; 
    
    lc.publish(params.CONTROL_CHANNEL, control_msg);
end