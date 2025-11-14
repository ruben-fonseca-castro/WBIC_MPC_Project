function publish_control_command(lc, control_msg, tau_cmd, contact_state, params, q_j_cmd, q_j_vel_cmd)
    % --- MODIFIED LINE: Added q_j_cmd and q_j_vel_cmd ---

    % This function fills and publishes the final LCM control message.
    % It now sends all 3 components:
    % 1. qj_pos = q_j^cmd (Desired Position)
    % 2. qj_vel = \dot{q}_j^cmd (Desired Velocity)
    % 3. qj_tau = tau_j (Feedforward Torque)
    
    control_msg.timestamp = int64(java.lang.System.currentTimeMillis() * 1000);
    
    % --- Feedforward Torque ---
    control_msg.qj_tau = tau_cmd;   
    
    % --- NEW: Set desired position and velocity for the PD controller ---
    control_msg.qj_pos = q_j_cmd;
    control_msg.qj_vel = q_j_vel_cmd;

    % --- Set gains for the Python PD controller ---
    control_msg.kp = params.KP_vec; 
    control_msg.kd = params.KD_vec;
    
    control_msg.contact = contact_state; 
    
    lc.publish(params.CONTROL_CHANNEL, control_msg);
end