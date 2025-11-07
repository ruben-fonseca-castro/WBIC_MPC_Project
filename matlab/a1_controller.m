clc; 
clear; 

%% 1. Setup Java Paths (Cross-Platform)
% This block detects the OS and sets the correct paths.
% Windows users: Please edit the 'lcm_lib_path' and 'project_dir'
% variables in the 'elseif ispc' section below.

if isunix
    % --- Ubuntu / Linux Paths ---
    disp('Linux/Unix system detected.');
    
    % Path to the main lcm.jar library
    lcm_lib_path = '/home/davidho/miniconda3/envs/arcpy/lib/python3.9/site-packages/share/java/lcm.jar';
    
    % Path to your project directory
    project_dir = fullfile(getenv('HOME'), 'arc-bridge');

elseif ispc
    % --- Windows Paths ---
    disp('Windows system detected.');
    
    % !!! WINDOWS USERS: EDIT THESE TWO LINES !!!
    
    % 1. Set the path to your 'lcm.jar' file
    % e.g., 'C:\lcm\lcm-1.4.0\lcm.jar'
    lcm_lib_path = 'C:\path\to\your\lcm.jar';
    
    % 2. Set the path to your 'arc-bridge' project folder
    % e.g., 'C:\Users\YourName\Documents\arc-bridge'
    project_dir = fullfile(getenv('USERPROFILE'), 'Documents', 'arc-bridge');
    
    % --- END OF WINDOWS EDIT SECTION ---
    
else
    error('Unsupported operating system. Please add paths manually.');
end

% --- Add the paths to the Java Classpath ---
% Use 'fullfile' to safely build OS-specific paths (handles / vs \)
a1_types_path = fullfile(project_dir, 'arc_bridge', 'lcm_msgs', 'lcm_msgs.jar');

% Add the paths (with a check to avoid warnings)
current_path = javaclasspath('-dynamic');

% Check and add main LCM library
if ~any(strcmp(current_path, lcm_lib_path))
    javaaddpath(lcm_lib_path);
    disp('Added LCM library to path.');
else
    % disp('LCM library already on path.'); % Silenced warning
end

% Check and add A1 message types
if ~any(strcmp(current_path, a1_types_path))
    javaaddpath(a1_types_path);
    disp('Added A1 message types to path.');
else
    % disp('A1 message types already on path.'); % Silenced warning
end


%% 2. Setup LCM
% !!! IMPORTANT: Replace with your actual channel names !!!
STATE_CHANNEL = 'A1_STATE'; 
CONTROL_CHANNEL = 'A1_CONTROL';

lc = lcm.lcm.LCM.getSingleton();

% Subscribe to state messages
lcm_agg_state = lcm.lcm.MessageAggregator();
lcm_agg_state.setMaxMessages(1); % Only store the latest message
lc.subscribe(STATE_CHANNEL, lcm_agg_state);

%% 3. Setup Controller
control_freq = 1000; % Set your control frequency (e.g., 1000 Hz)
dt = 1.0 / control_freq; % Calculate desired time per loop

% Create a re-usable control message object
control_msg = lcm_msgs.unitree_a1_control_t();

% --- Define your desired standing pose (from a1.xml keyframe) ---
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
    loop_start_time = tic; % Start the loop timer
    
    % --- Get Latest State ---
    msg = lcm_agg_state.getNextMessage(0); 
    if isempty(msg)
        % --- Wait for next cycle if no message ---
        elapsed_time = toc(loop_start_time);
        time_to_wait = dt - elapsed_time;
        if time_to_wait > 0
            pause(time_to_wait);
        end
        continue;
    end
    
    % De-serialize the Java message into a MATLAB struct
    state = lcm_msgs.unitree_a1_state_t(msg.data);

    % --- Controller Logic ---
    q_pos_curr = state.qj_pos;
    q_vel_curr = state.qj_vel;

    pos_error = q_des_home - q_pos_curr;
    vel_error = q_vel_des - q_vel_curr;
    
    tau_ff = zeros(12, 1); 
    tau_pd = KP_vec .* pos_error + KD_vec .* vel_error;
    tau_cmd = tau_pd + tau_ff;

    % --- Publish Control Command ---
    control_msg.timestamp = int64(java.lang.System.currentTimeMillis() * 1000);
    control_msg.qj_tau = tau_cmd;   
    control_msg.kp = KP_vec;      
    control_msg.kd = KD_vec;      
    control_msg.contact = [1; 1; 1; 1]; 
    
    lc.publish(CONTROL_CHANNEL, control_msg);

    % --- Wait for the next control cycle ---
    elapsed_time = toc(loop_start_time);
    time_to_wait = dt - elapsed_time;
    if time_to_wait > 0
        pause(time_to_wait);
    end
end