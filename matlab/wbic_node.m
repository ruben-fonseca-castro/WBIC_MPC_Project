clc; 
clear; 
format compact;

addpath('utils/');
addpath('controllers/');

%% 1. Setup
disp('Setting up paths...');
person_select = 'David'; % ('David' or 'Ruben' or 'Pranav')
setup_paths(person_select);

disp('Initializing controller...');
% Use a struct to hold all controller parameters
params = initialize_controller_state();

disp('Setting up LCM...');
% Pass params in, get all 4 aggregators out
[lc, agg_state, agg_joy, agg_plan] = setup_lcm(params); % <-- **MODIFIED**

disp('MATLAB Controller Running...');

%% 2. Main Control Loop
while true
    loop_start_time = tic; % Start the loop timer
    
    % --- 2a. Read all incoming LCM messages ---
    % Pass all aggregators and last states in
    [state, params.joy_state, params.mpc_plan, new_data] = read_lcm_messages(agg_state, agg_joy, agg_plan, ...
                          params.joy_state, params.mpc_plan);
    
    if ~new_data.state_received
        % If no new state message, wait and skip the rest of the loop
        elapsed_time = toc(loop_start_time);
        time_to_wait = params.dt - elapsed_time;
        if time_to_wait > 0
            pause(time_to_wait);
        end
        continue;
    end

    % --- 2b. Run the "Brain" (Controller Logic) ---
    % This logic now has access to the latest state, joystick, AND plan
    % because they are all in the 'params' struct.
    [tau_cmd, contact_state, params] = run_wbc_controller(state, params);
    
    % --- 2c. Publish the command ---
    publish_control_command(lc, params.control_msg, tau_cmd, contact_state, params);

    % --- 2d. Wait for the next control cycle ---
    elapsed_time = toc(loop_start_time);
    time_to_wait = params.dt - elapsed_time;
    if time_to_wait > 0
        pause(time_to_wait);
    end
end