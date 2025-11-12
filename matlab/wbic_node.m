clc; 
clear; 
format compact;

addpath('utils/');
addpath('controllers/');

disp('--- WBIC Controller ---');

%% 1. Setup
disp('Setting up paths...');
person_select = 'David'; % ('David' or 'Ruben' or 'Pranav')
setup_paths(person_select);

disp('Initializing controller...');
% Use a struct to hold all controller parameters
params = initialize_controller_state();

disp('Setting up LCM...');
% Pass params in, get all 4 aggregators out
[lc, agg_state, agg_joy, agg_plan] = setup_lcm(params);

disp('MATLAB Controller Running...');

%% 2. Main Control Loop
while true
    loop_start_time = tic; % Start the loop timer
    
    % --- a. Read all incoming LCM messages ---
    
    [state, params.joy_state, params.mpc_plan, new_data] = read_lcm_messages(agg_state, agg_joy, agg_plan, params.joy_state, params.mpc_plan);
    % agg = aggregator -- an lcm helper object that only store "aggregate" the most recent message

    if ~new_data.state_received
        % If no new state message, wait and skip the rest of the loop
        elapsed_time = toc(loop_start_time);
        time_to_wait = params.dt - elapsed_time;
        if time_to_wait > 0
            pause(time_to_wait);
        end
        continue;
    end

    % --- b. Run the WBIC Controller ---
    [tau_cmd, contact_state, params] = run_wbic_controller(state, params);
    
    % --- c. Publish the command ---
    publish_control_command(lc, params.control_msg, tau_cmd, contact_state, params);

    % --- d. Wait for the next control cycle ---
    elapsed_time = toc(loop_start_time);
    time_to_wait = params.dt - elapsed_time;
    if time_to_wait > 0
        pause(time_to_wait);
    end
end