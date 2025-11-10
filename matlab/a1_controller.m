clc; 
clear; 
format compact;

% Add the current folder to the MATLAB path (so it can find the functions)
addpath(genpath(pwd));

%% 1. Setup
disp('Setting up paths...');
person_select = 'David'; % ('David' or 'Ruben' or 'Pranav')
setup_paths(person_select);

disp('Setting up LCM...');
[lc, agg_state, agg_joy] = setup_lcm();

disp('Initializing controller...');
% Use a struct to hold all controller parameters
params = initialize_controller_state();

disp('MATLAB Controller Running...');

%% 2. Main Control Loop
while true
    loop_start_time = tic; % Start the loop timer
    
    % --- 2a. Read all incoming LCM messages ---
    [state, params.joy_state, new_data] = read_lcm_messages(agg_state, agg_joy, params.joy_state);
    
    if ~new_data.state_received
        % If no new state message, wait and skip the rest of the loop
        elapsed_time = toc(loop_start_time);
        time_to_wait = params.dt - elapsed_time;
        if time_to_wait > 0
            pause(time_to_wait);
        end
        continue;
    end

    % --- 2b. Run the Controller ---
    [tau_cmd, contact_state] = run_wbc_controller(state, params);
    % [tau_cmd, contact_state] = run_pd_controller(state, params);

    % --- 2c. Publish the command ---
    publish_control_command(lc, params.control_msg, tau_cmd, contact_state, params);

    % --- 2d. Wait for the next control cycle ---
    elapsed_time = toc(loop_start_time);
    time_to_wait = params.dt - elapsed_time;
    if time_to_wait > 0
        pause(time_to_wait);
    end
end