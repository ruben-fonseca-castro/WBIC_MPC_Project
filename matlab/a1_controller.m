clc; 
clear; 

%% 1. Setup Java Paths
% Add the main LCM library
javaaddpath('/home/davidho/miniconda3/envs/arcpy/lib/python3.9/site-packages/share/java/lcm.jar'); 

% Add your A1 message definitions
home_dir = getenv('HOME');
jar_path = fullfile(home_dir, 'arc-bridge', 'arc_bridge', 'lcm_msgs', 'lcm_msgs.jar');
javaaddpath(jar_path);

%% 2. Setup LCM
% !!! IMPORTANT: Replace with your actual channel names !!!
STATE_CHANNEL = 'A1_STATE'; 
CONTROL_CHANNEL = 'A1_CONTROL';

% Use the explicit UDPM address like your instructor
lc = lcm.lcm.LCM.getSingleton();
% lc = lcm.lcm.LCM('udpm://239.255.76.67:7667?ttl=1'); % Or use this

% Subscribe to state messages
lcm_agg_state = lcm.lcm.MessageAggregator();
lcm_agg_state.setMaxMessages(1); % Only store the latest message
lc.subscribe(STATE_CHANNEL, lcm_agg_state);

%% 3. Setup Controller
control_freq = 1000; % Set your control frequency (e.g., 1000 Hz)
rate_ctrl = rateControl(control_freq);

% Create a re-usable control message object
control_msg = lcm_msgs.unitree_a1_control_t();

% --- Define your desired standing pose (from a1.xml keyframe) ---
% qpos="0 0 0.27 1 0 0 0 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8"
q_des_home = [0, 0.9, -1.8, ... % FR
              0, 0.9, -1.8, ... % FL
              0, 0.9, -1.8, ... % RR
              0, 0.9, -1.8]';   % RL
q_vel_des = zeros(12, 1);

% --- Set your PD Gains ---
kp = 20.0; % Proportional gain
kd = 1.0;  % Derivative gain

% Convert single gains into 12-element vectors
KP_vec = ones(12, 1) * kp;
KD_vec = ones(12, 1) * kd;


disp('MATLAB Controller Running...');
disp(['Listening for state on: ' STATE_CHANNEL]);
disp(['Publishing control on: ' CONTROL_CHANNEL]);

%% 4. Main Control Loop
while true
    % --- Get Latest State ---
    % Use the non-blocking getNextMessage(0)
    msg = lcm_agg_state.getNextMessage(0); 
    if isempty(msg)
        % If no new message, skip this loop iteration
        % and wait for the next clock cycle
        rate_ctrl.waitfor(); 
        continue;
    end
    
    % De-serialize the Java message into a MATLAB struct
    state = lcm_msgs.unitree_a1_state_t(msg.data);

    % --- Controller Logic ---
    % Parse current state from the message
    q_pos_curr = state.qj_pos;
    q_vel_curr = state.qj_vel;

    % Calculate PD torques
    pos_error = q_des_home - q_pos_curr;
    vel_error = q_vel_des - q_vel_curr;
    
    % This is your feed-forward gravity torque (if you had one)
    tau_ff = zeros(12, 1); 
    
    % This is your PD feedback torque
    tau_pd = KP_vec .* pos_error + KD_vec .* vel_error;
    
    % Final torque is feedback + feed-forward
    tau_cmd = tau_pd + tau_ff;

    % --- Publish Control Command ---
    control_msg.timestamp = int64(java.lang.System.currentTimeMillis() * 1000);
    control_msg.qj_tau = tau_cmd;   % Send the torques you calculated
    control_msg.kp = KP_vec;      % Send your gains
    control_msg.kd = KD_vec;      % Send your gains
    control_msg.contact = [1; 1; 1; 1]; % Your gait schedule
    
    lc.publish(CONTROL_CHANNEL, control_msg);

    % Wait for the next control cycle
    rate_ctrl.waitfor();
end