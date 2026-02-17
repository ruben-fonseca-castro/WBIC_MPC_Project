function [tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd, f_r_final] = run_wbic_controller(state, params, dt_mpc)
    % Runs the entire wbic controller based on current/incoming state, joystick, and mpc plan

    % Initialize variables

    tau_j = zeros(12,1); % 3 motors per leg, 4 legs --> 12 total feedforward torques
    q_j_cmd = state.qj_pos; % this seems to set the commanded angle to the current position of the joint, why?
    q_j_vel_cmd = zeros(12,1); % initializes all joint velocity commands to 0
    contact_state = zeros(4,1); % im assuming this stores whether a leg should be contacting the ground or not?
    f_r_final = zeros(12,1);  % Initialize final forces output as zero
    using_mock_plan = false;

    persistent q_j_cmd_accum;
    if isempty(q_j_cmd_accum)
        q_j_cmd_accum = state.qj_pos; 
    end
    
    % Debug Counter

    persistent wbic_cnt;
    if isempty(wbic_cnt), wbic_cnt = 0; end
    wbic_cnt = wbic_cnt + 1;
    do_print = (mod(wbic_cnt, 20) == 0); % Print every 0.5s

    % Persistent state for event-based contact detection

    persistent prev_contact_state; % basically same as static in cpp, holds the values bw function calls

    if isempty(prev_contact_state)
        prev_contact_state = ones(4, 1);  % Start in stance (1 = contact, 0 = floating)
    end

    try
        %% --- 1. Setup Dynamics ---

        q_pos_curr = state.qj_pos; % extract current position locally from the current state
        H = reshape(state.inertia_mat, [18, 18]); % extract mass matrix from the lcm state

        C = state.bias_force; % extract coriolis from lcm state                     
        J_c = reshape(state.J_gc, [12, 18]); % extract contact jacobian, wtf is this again??      
        
        H_f = H(1:6, :); H_ff = H(1:6, 1:6); H_j = H(7:18, :); % partitions the mass matrix into 3 smaller matrices for later use       
        C_f = C(1:6); C_j = C(7:18); % separates the bias force for the centroid dof (C_f), and joint bias (C_j)         
        JcT_f = J_c(:, 1:6)'; JcT_j = J_c(:, 7:18)'; % gets the contact jacobian transpose for the centroid/free dof (f), and joint (j)   

        %% --- 2. Setup MPC Plan (MOCK / STANDALONE) --- 

        if ~isjava(params.mpc_plan) || isempty(params.mpc_plan) % if the mpc plan doesn't exist/is empty, create a mock mpc plan for wbic to follow
            using_mock_plan = true;
            MASS = 12.45; 
            GRAVITY = 9.81;
            mock_plan = struct();
            mock_plan.contact = [1;1;1;1]; % mock plan commands all legs standing

            % Calculate center of support (global frame)

            feet_x = state.p_gc([1, 4, 7, 10]); %takes the current position of the ground contact feet for all legs, x components
            feet_y = state.p_gc([2, 5, 8, 11]); %takes the current position of the ground contact feet for all legs, y components
            center_x = mean(feet_x); % takes the mean of the x's for the center x
            center_y = mean(feet_y); % takes the mean of the y's for the center y

            % Target: Centered over feet at specific height
            
            height = 0.3; %global frame, arbitrary, this does respond to this, so we good

            mock_plan.body_pos_cmd = [center_x; center_y; height]; % mock mpc global body xyz target coords
            mock_plan.body_rpy_cmd = [0;0;0]; % body roll pitch yaw [radians] in global frame target, all 0 for mock
            mock_plan.body_vel_cmd = zeros(3,1); % global frame velocity of centroid/body, 0 for mock
            mock_plan.body_omega_cmd = zeros(3,1); % [rad/s] rotational velocities in body frame, 0 for mock
            mock_plan.foot_pos_cmd = state.p_gc; % global frame sets foot position target to current foot position xyz for each leg (12 total)
            mock_plan.foot_vel_cmd = zeros(12,1); % global frame velocity targets for each foot --> 0 for all

            % --- Compute reaction forces using QP for Mock MPC ---

            % This distributes forces to achieve equilibrium + orientation correction

            p_com = state.position; % extracts current com position in global frame
            p_feet = reshape(state.p_gc, [3, 4]); % global frame foot positions, each row is x,y,z, each column feet 1-4

            % Build force distribution matrix
            % [sum of forces = mg] and [sum of moments = desired moment]

            A_fd = zeros(6, 12); % wrench map matrix, rows 1-3 total force from all feet, rows 4-6 total moments about COM

            for i = 1:4 % for each leg
                idx = (i-1)*3 + (1:3);
                A_fd(1:3, idx) = eye(3);  % Force sum, each leg contributes its own force to total
                r_i = p_feet(:, i) - p_com; % positional vector from com to a given foot, in global
                A_fd(4:6, idx) = skew_matrix(r_i);  % Moment sum, each foot contributes a moment based on vector from COM
            end

            % Desired wrench: gravity compensation + orientation correction
            % Add moment to correct pitch/roll errors (negative feedback)

            kp_ori = 75;  % Orientation correction gain, does changing this affect the SS pos of doggy?
            % Negative sign: if pitched forward (+), apply backward moment (-)

            % sets the desired moment (corrective) as a P controller with current state roll and pitch, and always
            % setting yaw to 0. Will only ever have a potential desired moment along roll and pitch directions

            % besides, dog still pitches up a bit, although much less noticeable, barely see wack deviations in the log anymore

            desired_moment = -kp_ori * [state.rpy(1); state.rpy(2); 0]; %maybe adjust this?? SS error most likely exist
            

            b_fd = [0; 0; MASS * GRAVITY; desired_moment]; % [6,1], Fz,Fy,Fz,Mr,Mp,My

            % QP to find forces: min ||f||^2 s.t. A*f = b, friction cone

            H_fd = eye(12); % spreads loads evenly among legs
            f_fd = zeros(12, 1); % causes QP to prefer small forces

            % Friction cone constraints (inequality constraints), A_ineq * x <= b_ineq 
            % this code actually dowuble flips the negative, but still works to enforce friction cone

            mu = 0.6;
            W_leg = [-1 0 mu; 1 0 mu; 0 -1 mu; 0 1 mu; 0 0 1]; % weights for inequality wrt leg force
            A_ineq_fd = -blkdiag(W_leg, W_leg, W_leg, W_leg); % builds a huge matrix for each leg
            b_ineq_fd = zeros(20, 1); % b_ineq

            % Solve the QP, min_f s.t. 1/2 f^T * H_fd * f, eq: A_fd * f = b_fd, ineq: A_ineq * f <= b_ineq

            opts = optimoptions('quadprog', 'Display', 'off'); % sets options for solver
            [f_sol, ~, exitflag] = quadprog(H_fd, f_fd, A_ineq_fd, b_ineq_fd, A_fd, b_fd, [], [], [], opts); % solves the bloody thing

            if exitflag == 1 % if QP optimal, use the solution
                
                mock_plan.reaction_force = f_sol; % sets plan reaction force equal to force solve

            else % otherwise, Fallback to equal forces
                
                f_z = (MASS * GRAVITY) / 4; % trivial distribution of forces among legs
                mock_plan.reaction_force = repmat([0; 0; f_z], 4, 1); %gives each leg 1/4 of gravity to deal w along z axis
                if do_print
                    fprintf('[WBIC Standalone] Force distribution QP failed, using equal forces\n');
                end
            end

            params.mpc_plan = mock_plan; % sets the mpc to the mock plan
        end

        % MPC done, moving into WBIC/Lower Level


        % %% --- 2a. Smooth MPC Trajectories (Dead Reckoning) --- I dont think this is helping so disabling
        % % Project the low-frequency MPC plan forward using velocity commands
        % % to prevent "staircase" inputs to the high-frequency WBIC.
        
        % if dt_mpc > 0 && ~using_mock_plan && isstruct(params.mpc_plan)
        %     % 1. Body Position Prediction: p_new = p_old + v * dt
        %     params.mpc_plan.body_pos_cmd = params.mpc_plan.body_pos_cmd + ...
        %                                 params.mpc_plan.body_vel_cmd * dt_mpc;

        %     % 2. Body Orientation Prediction: R_new approx R_old + (omega x R) * dt
        %     % Simple Euler integration for small timesteps:
        %     params.mpc_plan.body_rpy_cmd = params.mpc_plan.body_rpy_cmd + ...
        %                                 params.mpc_plan.body_omega_cmd * dt_mpc;

        %     % 3. Foot Position Prediction (Crucial for smooth swing)
        %     % Reshape, update, and reshape back
        %     foot_pos = reshape(params.mpc_plan.foot_pos_cmd, [3, 4]);
        %     foot_vel = reshape(params.mpc_plan.foot_vel_cmd, [3, 4]);
            
        %     % Only update feet that are moving (optional check, but vel should be 0 for stance anyway)
        %     foot_pos_updated = foot_pos + foot_vel * dt_mpc;
            
        %     params.mpc_plan.foot_pos_cmd = foot_pos_updated(:);
        % end


        %% --- 2a. Smooth MPC Trajectories (2nd Order Dead Reckoning) ---
        % Project prediction forward using Velocity AND Acceleration.
        
        if dt_mpc > 0 && ~using_mock_plan && isstruct(params.mpc_plan)
            % Body prediction (Keep existing)
            params.mpc_plan.body_pos_cmd = params.mpc_plan.body_pos_cmd + ...
                                        params.mpc_plan.body_vel_cmd * dt_mpc;
            params.mpc_plan.body_rpy_cmd = params.mpc_plan.body_rpy_cmd + ...
                                        params.mpc_plan.body_omega_cmd * dt_mpc;

            % [FIX] Foot Prediction: Pos = P0 + V0*t + 0.5*a*t^2
            % This creates a smooth arc instead of a straight line
            foot_pos = reshape(params.mpc_plan.foot_pos_cmd, [3, 4]);
            foot_vel = reshape(params.mpc_plan.foot_vel_cmd, [3, 4]);
            foot_acc = reshape(params.foot_acc_cmd, [3, 4]); % Calculated in wbic_node
            
            foot_pos_updated = foot_pos + foot_vel * dt_mpc + 0.5 * foot_acc * dt_mpc^2;
            
            % [FIX] ALSO update velocity: Vel = V0 + a*t
            % This eliminates the "staircase" velocity seen by the swing controller
            foot_vel_updated = foot_vel + foot_acc * dt_mpc;
            
            params.mpc_plan.foot_pos_cmd = foot_pos_updated(:);
            params.mpc_plan.foot_vel_cmd = foot_vel_updated(:); % Update the velocity too!
       end




        f_r_mpc = params.mpc_plan.reaction_force; % extract planned reaction force into local
        contact_cmd = params.mpc_plan.contact; % extract contact plan into local
        contact_state = contact_cmd;

        p_gc_curr = reshape(state.p_gc, [3, 4]); % gets current ground contact global position for each leg




        % %% --- 2b. Event-Based Contact Detection (Bledt et al. 2018) ---
        % % Use actual force feedback (state.foot_force) to detect early touch-down
        % % during swing: if the MPC commands swing but the foot already has measurable
        % % ground contact force, switch to stance for that leg.

        % force_threshold = 21;  % N - threshold for touch-down detection
        % contact_state = zeros(4, 1);

        % for leg = 1:4
        %     % Use measured/estimated foot force from robot state (not planned force)
        %     foot_force_actual = state.foot_force(leg);

        %     % Event-based override: MPC says swing but actual force above threshold -> early touch-down, ideally this shouldn't happen often right?
        %     if contact_cmd(leg) == 0 && foot_force_actual > force_threshold
        %         contact_state(leg) = 1;  % Early touch-down detected
        %     else
        %         contact_state(leg) = contact_cmd(leg);
        %     end
        % end


        % %% --- 2b. Event-Based Contact Detection (FIXED) ---
        % force_threshold = 20;  % N
        % contact_state = zeros(4, 1);
        
        % % Extract planned foot velocity (Z-component)
        % v_foot_plan = reshape(params.mpc_plan.foot_vel_cmd, [3, 4]);

        % for leg = 1:4
        %     foot_force_actual = state.foot_force(leg);
            
        %     % Check if MPC wants us to lift (Velocity Z > 0.1 m/s)
        %     is_lifting = v_foot_plan(3, leg) > 0.1;

        %     % LOGIC: 
        %     % 1. If MPC says Swing (0)...
        %     % 2. AND we are NOT trying to lift off (is_lifting == false)...
        %     % 3. AND force is high...
        %     % THEN -> It is an early landing.
            
        %     if contact_cmd(leg) == 0 && foot_force_actual > force_threshold && ~is_lifting
        %          contact_state(leg) = 1; 
        %     else
        %          contact_state(leg) = contact_cmd(leg);
        %     end
        % end












        % % Update previous contact state
        % prev_contact_state = contact_state;



        % Use trajectories from MPC (already generated with Bézier/stance modulation)

        p_gc_des = reshape(params.mpc_plan.foot_pos_cmd, [3, 4]); % extracts desired global foot positions from mpc

        try
            v_gc_des = reshape(params.mpc_plan.foot_vel_cmd, [3, 4]); % try to extract global foot EE velocities
        catch
            v_gc_des = zeros(3, 4); % set to zero if errors out??? does this happen a lot? print if so!!?
            fprintf('foot_vel set to zero!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        end
        
        q_dot_full = [state.velocity; state.omega; state.qj_vel]; % does this extract all the velocities in general?
        v_gc_act = J_c * q_dot_full; % jacobian transfer from joint space to task space

        %% --- 3. WEIGHTED SUM TASK CONTROL ---
        % All tasks are combined using a fixed weighted sum approach
        % (replaces null-space projection method)
        % Task priorities: Stance >> Orientation > Position > Swing
        
        % Paper Table I gains
        kp_base = 100; kd_base = 10;

        % Gain scheduling: Use different gains for standing vs. locomotion

        % Standing: Lower gains to avoid oscillations and noise amplification
        % Locomotion: Higher gains (Hyun et al. 2014) for aggressive tracking

        n_legs_in_contact = sum(contact_state); % add all leg states together

        if n_legs_in_contact == 4 % this directly checks all legs in contact, shouldn't this be handled more explicitity with a dynamics state that is set/maintained?
            % All feet in stance = standing mode 
            kp_foot = 100; kd_foot = 10;  % Original stable gains
            gait_mode_str = 'STAND';
        else
            % Dynamic gait (trot, etc)
            kp_foot = 450; kd_foot = 50;  % Hyun et al. (2014) aggressive gains
            gait_mode_str = 'TROT';
        end 


        % --- TASK 0: Stance Constraints ---
        % assuming these are stance constraints for legs that are in stance as opposed to swing

        J_stance = [];
        for i = 1:4
            if contact_state(i) == 1, J_stance = [J_stance; J_c(3*i-2 : 3*i, :)]; end
        end
        
        if ~isempty(J_stance)
            x_ddot_0 = zeros(size(J_stance, 1), 1);  % Zero acceleration for all stance legs
        else
            J_stance = zeros(0, 18);  % Empty matrix if all legs in swing, shouldn'this never happen??
            x_ddot_0 = zeros(0, 1); % x_ddot is also just empty?? is it used later on then?? why would it be zero
        end

        % --- TASK 1: Body Orientation ---

        J_1 = [zeros(3,3), eye(3), zeros(3,12)]; % creates a mega jacobian, columns 4-6 deal with orientation so those are active
        rot_err = params.mpc_plan.body_rpy_cmd - state.rpy; % subtracts mpc planned rpy from actual
        omega_err = params.mpc_plan.body_omega_cmd - state.omega; % subtracts planned rot vel from actual rot vel
        x_ddot_1 = 2 * kp_base * rot_err + 2 * kd_base * omega_err; % wait so, why is this multiying by zero?? no x_ddot influence
        
        % Store for debug
        ori_err_deg = rot_err * 180/pi; % converts to degrees


        % --- TASK 2: Body Position ---

        J_2 = [eye(3), zeros(3,3), zeros(3,12)]; % on position, so columns 1-3 only
        pos_err = params.mpc_plan.body_pos_cmd - state.position; % planned - actual body pos global
        vel_err = params.mpc_plan.body_vel_cmd - state.velocity; % planned - actual body vel global
        x_ddot_2 = kp_base * pos_err + kd_base * vel_err; % PD controller on the errors

        % Store for debug

        pos_err_m = pos_err;

        % --- TASK 3: Swing Foot ---

        
        % [FIX] Use the smooth acceleration we calculated in wbic_node
        a_gc_des = reshape(params.foot_acc_cmd, [3, 4]);

        J_swing = []; x_ddot_3 = []; 
        for i = 1:4 
            if contact_state(i) == 0 % Swing Leg
                J_swing = [J_swing; J_c(3*i-2 : 3*i, :)]; 
                idx = 3*i-2 : 3*i; 
                
                p_err = p_gc_des(:, i) - p_gc_curr(:, i); 
                v_err = v_gc_des(:, i) - v_gc_act(idx); 
                
                % [FIX] Feedforward Acceleration
                % Now a_gc_des is non-zero (approx 10-15 m/s^2), which provides 
                % the inertial "kick" needed to lift the leg without lag.
                acc_swing = kp_foot * p_err + kd_foot * v_err + a_gc_des(:, i); 
                
                x_ddot_3 = [x_ddot_3; acc_swing]; 
            end
        end


        % % [NEW] Persistent variable to calculate feedforward acceleration
        % persistent v_gc_des_prev;
        % if isempty(v_gc_des_prev), v_gc_des_prev = zeros(3,4); end

        % % Calculate Acceleration: (v_next - v_prev) / dt
        % a_gc_des = (v_gc_des - v_gc_des_prev) / params.dt;
        % v_gc_des_prev = v_gc_des; % Update for next loop

        % J_swing = []; x_ddot_3 = []; % initialize variables to fill in later
        % for i = 1:4 % each leg
        %     if contact_state(i) == 0 % if swinging
        %         J_swing = [J_swing; J_c(3*i-2 : 3*i, :)]; % append contact jacobian splice for specific leg
        %         idx = 3*i-2 : 3*i; % shoudln't this go before the above line to simplify calcs?
        %         p_err = p_gc_des(:, i) - p_gc_curr(:, i); % positional error of "ground contact" for each leg, is this wokring as intended?
        %         v_err = v_gc_des(:, i) - v_gc_act(idx); % vel error of ground contact leg, again, is this how it shoul be done?

        %         % %the old method
        %         % acc_swing = kp_foot * p_err + kd_foot * v_err; % PD controller for acc of swinging, seems kinda TSC'y

        %         % [FIX] Add Feedforward Acceleration (a_gc_des)
        %         % Now the leg 'knows' it needs to accelerate, even with 0 error.
        %         acc_swing = kp_foot * p_err + kd_foot * v_err + a_gc_des(:, i);


        %         x_ddot_3 = [x_ddot_3; acc_swing]; % append the acceleration
        %     end
        % end
        
        if isempty(J_swing) % if all legs are on the ground
            J_swing = zeros(0, 18);  % Empty matrix if no swing legs
            x_ddot_3 = zeros(0, 1); % empty accel
        end


        % --- WEIGHTED SUM SOLUTION ---

        % Stack all tasks: J_all * q_ddot = x_ddot_all


        J_all = [J_stance; J_1; J_2; J_swing]; % vert cat all the jacobians from before, again confused over J_stance and J_swing??
        x_ddot_all = [x_ddot_0; x_ddot_1; x_ddot_2; x_ddot_3]; % vert cat all the accelerations
        
        % Priority weights: higher weight = higher priority
        % Stance gets very high weight (almost hard constraint)
        % Orientation > Position > Swing
        % w_stance = 10000;   % Very high priority for stance constraints
        % w_orientation = 2000;
        % w_position = 2000;
        % w_swing = 1000;

        % Gemini recommended weights
        % w_stance = 2000;      % Relax this slightly
        % w_swing  = 5000;      % Prioritize SWING tracking over perfect stance clamping
        % w_orientation = 1200;
        % w_position = 500;

        w_stance = 10000;
        w_orientation = 2000;
        w_position = 1500;
        w_swing = 5000;      % Let the IK handle the swing tracking!

        
        % Build diagonal weight matrix
        n_stance = size(J_stance, 1);
        n_orientation = 3;
        n_position = 3;
        n_swing = size(J_swing, 1);
        
        W = diag([w_stance * ones(n_stance, 1); ...
                  w_orientation * ones(n_orientation, 1); ...
                  w_position * ones(n_position, 1); ...
                  w_swing * ones(n_swing, 1)]);
        
        % Solve weighted least squares: min ||W^(1/2) * (J_all * q_ddot - x_ddot_all)||^2, !!why do we used weighted least squares again?
        % Using regularization for numerical stability

        reg = 1e-6;  % Small regularization term

        q_ddot_cmd = (J_all' * W * J_all + reg * eye(18)) \ (J_all' * W * x_ddot_all); % final joint commanded accelerations


        % %% --- 3b. Joint Integration (Paper Style) ---
        % % Integrate acceleration for ONE step, then add to MEASURED state.
        % % This prevents drift and "windup".

        % dt = params.dt;
        % q_j_acc = q_ddot_cmd(7:18);

        % % 1. Velocity Command: Predicted velocity for next step
        % q_j_vel_cmd = state.qj_vel + q_j_acc * dt;

        % % 2. Position Command: Measured Position + Kinematic Delta
        % % Paper Eq. 24: q_cmd = q_meas + delta_q
        % q_j_cmd = state.qj_pos + q_j_vel_cmd * dt;

        

        % % Joint Integration, the og one

        % dt_wbic = params.dt; % uses the dt from the params struct, which is 0.001
        % q_j_acc = q_ddot_cmd(7:18); % extracts the joint acclerations from the mega accleration command
        % % q_j_vel_cmd = state.qj_vel + q_j_acc * dt_wbic; % commanded vel is current vel + accel * dt
        % % q_j_cmd = state.qj_pos + q_j_vel_cmd * dt_wbic; % commanded position is current pos + commanded vel * dt

        % q_j_vel_cmd = state.qj_vel + q_j_acc * dt_wbic;
        % q_j_cmd     = state.qj_pos + state.qj_vel * dt_wbic + 0.5 * q_j_acc * dt_wbic^2;




        % % --- Joint Integration (REPLACED) ---

        % q_j_acc = q_ddot_cmd(7:18);

        % % Integrate the COMMANDED velocity, not the STATE velocity.
        % % This allows the command to "pull ahead" of the robot if the robot lags.
        % % Note: You usually want a decay or sync term to prevent drift if motors saturate.
        % alpha = 0.1; % Blend factor (0.1 = mostly integrate command, but pull slightly to reality)

        % % Ideally: q_next = q_prev + q_vel_cmd * dt + 0.5 * acc * dt^2
        % % But since we don't have q_vel_cmd state here, we update:

        % q_j_vel_cmd = state.qj_vel + q_j_acc * params.dt; % This is still just a 1-step prediction

        % % BETTER: Use the Task Error to drive the position command directly
        % % For swing legs, map Cartesian Desired Pos -> Joint Desired Pos (Analytical IK is best here)
        % % For now, we can try simply NOT resetting the base to 'state.qj_pos' fully.

        % % Simple accumulation (Allow error to build up so Kp kicks in):
        % q_j_cmd_accum = q_j_cmd_accum + state.qj_vel * params.dt + 0.5 * q_j_acc * params.dt^2;

        % % Correction to prevent infinite drift (optional)
        % q_j_cmd_accum = q_j_cmd_accum + 0.05 * (state.qj_pos - q_j_cmd_accum); 

        % q_j_cmd = q_j_cmd_accum;
        % q_j_vel_cmd = state.qj_vel + q_j_acc * params.dt;




        % %% --- 3b. Joint Integration (CORRECTED) --- another version
        % % We must integrate ACCELERATION -> VELOCITY -> POSITION
        % % Do NOT reset to 'state.qj_vel' or 'state.qj_pos' every step, 
        % % or you kill the motor stiffness.

        % persistent q_j_vel_cmd_accum;
        
        % % Initialization (First run only)
        % if isempty(q_j_vel_cmd_accum)
        %     q_j_vel_cmd_accum = zeros(12, 1); 
        % end
        
        % % If we just switched modes or startup, maybe sync velocity (optional safety)
        % % But generally, trust the integrator.

        % q_j_acc = q_ddot_cmd(7:18); % Extract joint accelerations
        % dt = params.dt;

        % % 1. Integrate Velocity: v_cmd = v_old + a_cmd * dt
        % q_j_vel_cmd_accum = q_j_vel_cmd_accum + q_j_acc * dt;
        
        % % Decay/Saturation for safety (prevent windup if bot falls)
        % % Slowly bleed velocity to zero if it gets crazy high, but don't fight motion
        % q_j_vel_cmd_accum = q_j_vel_cmd_accum * 0.999; 

        % % 2. Integrate Position: p_cmd = p_old + v_cmd * dt
        % % q_j_cmd_accum was initialized at top of file
        % q_j_cmd_accum = q_j_cmd_accum + q_j_vel_cmd_accum * dt;

        % % 3. Set Final Output Variables
        % q_j_vel_cmd = q_j_vel_cmd_accum;
        % q_j_cmd = q_j_cmd_accum;

        % % [CRITICAL]: Remove the "leak" to state.qj_pos. 
        % % If you add + 0.05 * (state.qj_pos - q_j_cmd), you disable the Kp gain.
        % % Only sync if error is MASSIVE (safety reset), otherwise trust command.
        % if max(abs(state.qj_pos - q_j_cmd)) > 0.5 % ~30 degrees error
        %      % Emergency sync if we are totally desynced
        %      q_j_cmd_accum = state.qj_pos;
        %      q_j_vel_cmd_accum = state.qj_vel;
        % end




        % %% --- 3b. Stabilized Joint Integration ---
        % persistent q_j_vel_cmd_accum;
        % if isempty(q_j_vel_cmd_accum), q_j_vel_cmd_accum = state.qj_vel; end
        
        % dt = params.dt;
        % q_j_acc = q_ddot_cmd(7:18);

        % % 1. Velocity Integration with High Damping
        % % We blend the integrated command with the actual measured velocity 
        % % to prevent the 'velocity' command from running away.
        % vel_alpha = 0.1; % 10% Trust reality, 90% Trust command
        % q_j_vel_cmd_accum = (1 - vel_alpha) * (q_j_vel_cmd_accum + q_j_acc * dt) + ...
        %                     (vel_alpha) * state.qj_vel;

        % % 2. Position Integration with "Leaky" Spring
        % % This is the key: we allow q_j_cmd to be different from state.qj_pos,
        % % but we 'pull' it toward reality just enough to stop oscillations.
        % pos_alpha = 0.05; 
        % q_j_cmd_accum = q_j_cmd_accum + q_j_vel_cmd_accum * dt;
        
        % % The "Tether": Prevents the command from oscillating away from the robot
        % q_j_cmd_accum = q_j_cmd_accum + pos_alpha * (state.qj_pos - q_j_cmd_accum);

        % q_j_vel_cmd = q_j_vel_cmd_accum;
        % q_j_cmd = q_j_cmd_accum;



        % %% --- 3b. Hybrid Joint Integration ---
        % % Stance: Reset to measured (Compliant / Paper Style)
        % % Swing:  Pure Integration (Stiff Tracking)
        
        % dt = params.dt;
        % q_j_acc = q_ddot_cmd(7:18);
        
        % % 1. Velocity Command
        % q_j_vel_cmd = state.qj_vel + q_j_acc * dt;
        
        % % 2. Position Command (Split Logic)
        % q_j_cmd = zeros(12, 1);
        
        % for leg = 1:4
        %     % Get indices for this leg's 3 joints
        %     idx = 3*leg-2 : 3*leg;
            
        %     if contact_state(leg) == 1
        %         % STANCE LEG: Paper Eq. 24
        %         % Reset to measured position to prevent drift fighting the ground
        %         q_j_cmd(idx) = state.qj_pos(idx) + q_j_vel_cmd(idx) * dt;
        %     else
        %         % SWING LEG: Pure Integration
        %         % Do NOT reset to measured. Trust the integral to build stiffness.
        %         % Use the accumulator we initialized earlier
        %         q_j_cmd_accum(idx) = q_j_cmd_accum(idx) + q_j_vel_cmd(idx) * dt;
        %         q_j_cmd(idx) = q_j_cmd_accum(idx);
        %     end
        % end
        
        % % Update the accumulator for stance legs too (to keep it synced for the next switch)
        % % If we don't sync this, the leg will snap when it enters swing phase.
        % for leg = 1:4
        %      if contact_state(leg) == 1
        %          idx = 3*leg-2 : 3*leg;
        %          q_j_cmd_accum(idx) = state.qj_pos(idx); % Sync accumulator to reality during stance
        %      end
        % end





        % %% --- 3b. Kinematic IK for Joint Commands (Paper Eq 16 & 17) ---
        % q_j_cmd = state.qj_pos;
        % q_j_vel_cmd = state.qj_vel;
        % dt = params.dt;

        % for leg = 1:4
        %     j_idx = 3*leg-2 : 3*leg;
            
        %     if contact_state(leg) == 1
        %         % STANCE: Compliant tracking (reset to measured)
        %         % q_j_cmd(j_idx) = state.qj_pos(j_idx);
        %         % q_j_vel_cmd(j_idx) = state.qj_vel(j_idx);

        %         % 1. Velocity Command comes from WBIC acceleration (Task 0: Stance = 0 accel)
        %         q_j_vel_cmd(j_idx) = state.qj_vel(j_idx) + q_ddot_cmd(6 + j_idx) * dt;
                 
        %         % 2. Position Command = Measured + Velocity * dt
        %         q_j_cmd(j_idx) = state.qj_pos(j_idx) + q_j_vel_cmd(j_idx) * dt;
        %     else
        %         % SWING: Direct Inverse Kinematics
        %         % Extract the 3x3 Jacobian specific to this leg's joints
        %         J_leg = J_c(j_idx, 6 + j_idx);
                
        %         % Cartesian errors
        %         p_err = p_gc_des(:, leg) - p_gc_curr(:, leg);
        %         v_err = v_gc_des(:, leg);
                
        %         % Map Cartesian error directly to joint error (Damped pseudo-inverse)
        %         delta_q = (J_leg' * J_leg + 1e-4 * eye(3)) \ (J_leg' * p_err);
                
        %         % Set commands so the Motor PD loop sees the FULL error
        %         q_j_cmd(j_idx) = state.qj_pos(j_idx) + delta_q;
                
        %         % Velocity IK
        %         q_j_vel_cmd(j_idx) = (J_leg' * J_leg + 1e-4 * eye(3)) \ (J_leg' * v_err);
        %     end
        % end


        % %% --- 3b. Robust Integration (The "Rigid Stance" Fix) ---
        % dt = params.dt; % Fixed 0.001
        
        % q_j_cmd = zeros(12, 1);
        % q_j_vel_cmd = zeros(12, 1);

        % for leg = 1:4
        %     idx = 3*leg-2 : 3*leg;
            
        %     % 1. Calculate Velocity Command for ALL legs
        %     %    (Stance legs have v~0, Swing legs have v~High)
        %     q_j_acc = q_ddot_cmd(6 + idx); % Extract joint accel
        %     q_j_vel_cmd(idx) = state.qj_vel(idx) + q_j_acc * dt;
            
        %     if contact_state(leg) == 1
        %         % STANCE: Integrate velocity (Do NOT reset to measured)
        %         % This keeps the "virtual spring" loaded. If the robot sinks,
        %         % q_cmd stays high, creating a large error -> High PD Torque.
                
        %         % Use the accumulator we initialized at the top of the function
        %         q_j_cmd_accum(idx) = q_j_cmd_accum(idx) + q_j_vel_cmd(idx) * dt;
                
        %         % Safety: Blend slightly to reality to prevent infinite drift (0.1% leak)
        %         q_j_cmd_accum(idx) = 0.999 * q_j_cmd_accum(idx) + 0.001 * state.qj_pos(idx);
                
        %         q_j_cmd(idx) = q_j_cmd_accum(idx);
                
        %     else
        %         % SWING: Kinematic IK (Your existing working code)
        %         % ... [Insert your existing IK logic here] ...
                
        %         % Update the accumulator so it's ready for the next landing
        %         q_j_cmd_accum(idx) = q_j_cmd(idx);
        %     end
        % end


        %% --- 3b. Joint Integration (Split Logic) ---
        dt_wbic = params.dt;
        q_j_acc = q_ddot_cmd(7:18); % Extract joint accelerations from WLS
        
        % 1. Calculate Velocity Command for ALL legs
        q_j_vel_cmd = state.qj_vel + q_j_acc * dt_wbic;
        
        % 2. Position Command (The Critical Fix)
        q_j_cmd = zeros(12, 1);
        
        for leg = 1:4
            idx = 3*leg-2 : 3*leg;
            
            if contact_state(leg) == 1
                % STANCE: Reset to Measured (Compliant)
                % This prevents the robot from trying to push through the floor
                q_j_cmd(idx) = state.qj_pos(idx) + q_j_vel_cmd(idx) * dt_wbic;
                
                % Sync the accumulator so it's ready when we switch to swing
                q_j_cmd_accum(idx) = state.qj_pos(idx); 
            else
                % SWING: Pure Integration (Stiff)
                % Do NOT reset to measured. Let q_cmd_accum pull away from reality.
                % This builds up the error (q_cmd - q_meas) needed for Kp to generate lift.
                
                % Integrate: pos = pos_old + vel * dt + 0.5 * acc * dt^2
                q_j_cmd_accum(idx) = q_j_cmd_accum(idx) + ...
                                     q_j_vel_cmd(idx) * dt_wbic + ...
                                     0.5 * q_j_acc(idx) * dt_wbic^2;
                
                q_j_cmd(idx) = q_j_cmd_accum(idx);
            end
        end







        %% --- 4. QP SOLVER FOR WBIC ---

        n_vars = 18; % 12 joints, 6 body dof
        % Q1 = 2 * eye(12); Q2 = 950 * eye(6); % quadratic costs, Q1 for reaction forces on each leg, xyz, Q2 for floating base acceleration
        Q1 = 1 * eye(12); Q2 = 0.1 * eye(6); % quadratic costs, Q1 for reaction forces on each leg, xyz, Q2 for floating base acceleration
        % ^ rn, reaction force tracking is prioritized over floating base accel tracking
        H_qp = 2 * blkdiag(Q2, Q1); f_qp = zeros(n_vars, 1); % buils the quadratic cost, multiplies by 2, sets linear cost to 0

        % Floating base dynamics constraint

        % NOTE: state.bias_force (C) already includes gravity from MuJoCo's qfrc_bias
        A_dyn = [H_ff, -JcT_f];
        b_dyn = (JcT_f * f_r_mpc) - (H_f * q_ddot_cmd) - C_f;

        % Swing foot zero force constraints

        A_swing_const = []; b_swing_const = [];

        for i = 1:4
            if contact_state(i) == 0
                f_idx_start = 6 + (i-1)*3 + 1;
                A_sub = zeros(3, 18); A_sub(1:3, f_idx_start:f_idx_start+2) = eye(3);
                A_swing_const = [A_swing_const; A_sub]; 


                % b_swing_const = [b_swing_const; 0; 0; 0];

                % Calculate the indices for this leg
                idx_leg = (i-1)*3 + 1 : (i-1)*3 + 3;

                % This says: "The change must oppose the plan so the sum is zero."
                % delta_f = -f_mpc  ==>  f_final = f_mpc + (-f_mpc) = 0
                b_swing_const = [b_swing_const; -f_r_mpc(idx_leg)];
            end
        end

        A_eq = [A_dyn; A_swing_const]; b_eq = [b_dyn; b_swing_const];

        % Friction cone ONLY for stance legs

        mu = 0.6;  % Match MPC friction coefficient
        W_leg = [ -1, 0, mu; 1, 0, mu; 0,-1, mu; 0, 1, mu; 0, 0, 1 ];

        A_ineq = []; b_ineq = [];

        for i = 1:4
            if contact_state(i) == 1  % Only stance legs
                idx = 3*i-2:3*i;  % Indices in f_r_mpc (1-3, 4-6, 7-9, 10-12)
                f_idx_global = 6 + idx;  % Indices in QP variable [delta_f(6); delta_fr(12)]

                % Create constraint matrix for this stance leg
                W_i = zeros(5, 18);
                W_i(:, f_idx_global) = -W_leg;  % Apply -W to delta_fr

                % RHS: W_leg * f_r_mpc for this leg
                b_i = W_leg * f_r_mpc(idx);

                A_ineq = [A_ineq; W_i];
                b_ineq = [b_ineq; b_i];
            end
        end

        % Solve QP using quadprog

        options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
        [x_sol, ~, flag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);

        if flag == 1 % if optimization successful
            delta_f = x_sol(1:6); delta_fr = x_sol(7:18);
            f_r_final = f_r_mpc + delta_fr;
            q_ddot_final = q_ddot_cmd; q_ddot_final(1:6) = q_ddot_final(1:6) + delta_f;
            tau_j = H_j * q_ddot_final + C_j - JcT_j * f_r_final;
        else
            tau_j = C_j - JcT_j * f_r_mpc;
            f_r_final = f_r_mpc;
            if do_print
                fprintf('[WBIC] ⚠ QP failed (flag=%d), using feedforward torques\n', flag);
            end
        end

        % ===================== WBIC DEBUG LOGGING =====================

        if do_print
            fprintf('\n================ WBIC DEBUG [%d] ================\n', wbic_cnt);
            fprintf('Mode: %s | Gains: kp=%d kd=%d | QP flag=%d %s\n', ...
                gait_mode_str, kp_foot, kd_foot, flag, wbic_qp_status(flag));
            fprintf('Contact (cmd/actual): [%d %d %d %d] / [%d %d %d %d]\n', ...
                contact_cmd, contact_state);

            fprintf('\n--- BODY ERRORS ---\n');
            fprintf('  Ori Err: [%.2f, %.2f, %.2f] deg (R,P,Y)\n', ori_err_deg);
            fprintf('  Pos Err: [%.4f, %.4f, %.4f] m\n', pos_err_m);
            fprintf('  Omega Err: [%.3f, %.3f, %.3f] rad/s\n', omega_err);
            fprintf('  Vel Err: [%.3f, %.3f, %.3f] m/s\n', vel_err);

            fprintf('\n--- TASK ACCELERATIONS ---\n');
            fprintf('  Ori x_ddot: [%.2f, %.2f, %.2f] rad/s²\n', x_ddot_1);
            fprintf('  Pos x_ddot: [%.2f, %.2f, %.2f] m/s²\n', x_ddot_2);

            fprintf('\n--- REACTION FORCES (N) ---\n');
            leg_names = {'FR', 'FL', 'RR', 'RL'};
            for leg = 1:4
                idx = 3*leg-2:3*leg;
                fprintf('  %s: MPC=[%6.1f,%6.1f,%6.1f] -> Final=[%6.1f,%6.1f,%6.1f]\n', ...
                    leg_names{leg}, f_r_mpc(idx), f_r_final(idx));
            end

            fprintf('\n--- JOINT TORQUES (Nm) ---\n');
            fprintf('  FR: [%6.2f, %6.2f, %6.2f]\n', tau_j(1:3));
            fprintf('  FL: [%6.2f, %6.2f, %6.2f]\n', tau_j(4:6));
            fprintf('  RR: [%6.2f, %6.2f, %6.2f]\n', tau_j(7:9));
            fprintf('  RL: [%6.2f, %6.2f, %6.2f]\n', tau_j(10:12));
            fprintf('  Max |tau|: %.2f Nm\n', max(abs(tau_j)));

            fprintf('\n--- JOINT COMMANDS ---\n');
            fprintf('  q_j_cmd (deg): [');
            fprintf('%.1f ', q_j_cmd * 180/pi);
            fprintf(']\n');

            fprintf('=================================================\n\n');
        end
        
    catch ME
         disp(['WBIC Error: ', ME.message]);
         fprintf('Line: %d\n', ME.stack(1).line);
    end
end

function str = wbic_qp_status(flag)
    switch flag
        case 1, str = '(optimal)';
        case 0, str = '(max iter)';
        case -2, str = '(infeasible)';
        case -3, str = '(unbounded)';
        otherwise, str = sprintf('(code: %d)', flag);
    end
end

function S = skew_matrix(v)
    % Returns the skew-symmetric matrix for cross product: S*x = v x x
    S = [  0,   -v(3),  v(2);
         v(3),    0,  -v(1);
        -v(2),  v(1),    0];
end