function [tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd, f_r_final] = run_wbic_controller(state, params)
    % Runs the entire wbic controller based on current/incoming state, joystick, and mpc plan

    % Initialize variables

    tau_j = zeros(12,1); % 3 motors per leg, 4 legs --> 12 total feedforward torques
    q_j_cmd = state.qj_pos; % this seems to set the commanded angle to the current position of the joint, why?
    q_j_vel_cmd = zeros(12,1); % initializes all joint velocity commands to 0
    contact_state = zeros(4,1); % im assuming this stores whether a leg should be contacting the ground or not?
    f_r_final = zeros(12,1);  % Initialize final forces output as zero
    
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
            
            height = 0.38; %global frame, arbitrary, this does respond to this, so we good

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

            kp_ori = 50;  % Orientation correction gain, does changing this affect the SS pos of doggy?
            % Negative sign: if pitched forward (+), apply backward moment (-)

            % sets the desired moment (corrective) as a P controller with current state roll and pitch, and always
            % setting yaw to 0. Will only ever have a potential desired moment along roll and pitch directions

            % besides, dog still pitches up a bit, although much less noticeable, barely see wack deviations in the log anymore

            desired_moment = -kp_ori * [state.rpy(1); state.rpy(2); 0]; % but honestly, shoulnd't his just be zero?? Test it!!!

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

        f_r_mpc = params.mpc_plan.reaction_force; % extract planned reaction force into local
        contact_cmd = params.mpc_plan.contact; % extract contact plan into local

        p_gc_curr = reshape(state.p_gc, [3, 4]); % gets current ground contact global position for each leg

        %% --- 2b. Event-Based Contact Detection (Bledt et al. 2018) ---
        % Use actual force feedback (state.foot_force) to detect early touch-down
        % during swing: if the MPC commands swing but the foot already has measurable
        % ground contact force, switch to stance for that leg.

        force_threshold = 20.0;  % N - threshold for touch-down detection
        contact_state = zeros(4, 1);

        for leg = 1:4
            % Use measured/estimated foot force from robot state (not planned force)
            foot_force_actual = state.foot_force(leg);

            % Event-based override: MPC says swing but actual force above threshold -> early touch-down, ideally this shouldn't happen often right?
            if contact_cmd(leg) == 0 && foot_force_actual > force_threshold
                contact_state(leg) = 1;  % Early touch-down detected
            else
                contact_state(leg) = contact_cmd(leg);
            end
        end

        % Update previous contact state

        prev_contact_state = contact_state;

        % Use trajectories from MPC (already generated with Bézier/stance modulation)

        p_gc_des = reshape(params.mpc_plan.foot_pos_cmd, [3, 4]); % extracts desired global foot positions from mpc

        try
            v_gc_des = reshape(params.mpc_plan.foot_vel_cmd, [3, 4]); % try to extract global foot EE velocities
        catch
            v_gc_des = zeros(3, 4); % set to zero if errors out??? does this happen a lot? print if so!!?
        end
        
        q_dot_full = [state.velocity; state.omega; state.qj_vel]; % does this extract all the velocities in general?
        v_gc_act = J_c * q_dot_full; % what does this do? 

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
            J_stance = zeros(0, 18);  % Empty matrix if all legs in swing
            x_ddot_0 = zeros(0, 1); % x_ddot is also just empty?? is it used later on then?? why would it be zero
        end

        % --- TASK 1: Body Orientation ---

        J_1 = [zeros(3,3), eye(3), zeros(3,12)]; % creates a mega jacobian, columns 4-6 deal with orientation so those are active
        rot_err = params.mpc_plan.body_rpy_cmd - state.rpy; % subtracts mpc planned rpy from actual
        omega_err = params.mpc_plan.body_omega_cmd - state.omega; % subtracts planned rot vel from actual rot vel
        x_ddot_1 = 0 * kp_base * rot_err + 0 * kd_base * omega_err; % wait so, why is this multiying by zero?? no x_ddot influence
        % CHECK ^^ IF THIS IS WHAT IS CAUSING THE PITCHING UP SHIT IN FULL MPC, EVEN DRIFTING IN WBIC ONLY MODE TOO
        % fprintf("nuking body orientation");
        
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

        J_swing = []; x_ddot_3 = []; % initialize variables to fill in later
        for i = 1:4 % each leg
            if contact_state(i) == 0 % if swinging
                J_swing = [J_swing; J_c(3*i-2 : 3*i, :)]; % append contact jacobian splice for specific leg
                idx = 3*i-2 : 3*i; % shoudln't this go before the above line to simplify calcs?
                p_err = p_gc_des(:, i) - p_gc_curr(:, i); % positional error of "ground contact" for each leg, is this wokring as intended?
                v_err = v_gc_des(:, i) - v_gc_act(idx); % vel error of ground contact leg, again, is this how it shoul be done?
                acc_swing = kp_foot * p_err + kd_foot * v_err; % PD controller for acc of swinging, seems kinda TSC'y
                x_ddot_3 = [x_ddot_3; acc_swing]; % append the acceleration
            end
        end
        
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
        w_stance = 1e6;   % Very high priority for stance constraints, why??
        w_orientation = 1000;
        w_position = 100;
        w_swing = 1;
        
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

        

        % Joint Integration

        % !!!Is this how integration works??? shouldnt it include the commanded accel for positional command too?? (1/2*a*t^2)

        dt_wbic = params.dt; % uses the dt from the params struct, which is 0.001
        q_j_acc = q_ddot_cmd(7:18); % extracts the joint acclerations from the mega accleration command
        q_j_vel_cmd = state.qj_vel + q_j_acc * dt_wbic; % commanded vel is current vel + accel * dt
        q_j_cmd = state.qj_pos + q_j_vel_cmd * dt_wbic; % commanded position is current pos + commanded vel * dt


        %% --- 4. QP SOLVER FOR WBIC ---

        n_vars = 18; % 12 joints, 6 body dof
        Q1 = 1.0 * eye(12); Q2 = 0.1 * eye(6); % what is this
        H_qp = 2 * blkdiag(Q2, Q1); f_qp = zeros(n_vars, 1); % what is this

        % Floating base dynamics constraint

        % NOTE: state.bias_force (C) already includes gravity from MuJoCo's qfrc_bias
        A_dyn = [H_ff, -JcT_f];
        b_dyn = (JcT_f * f_r_mpc) - (H_f * q_ddot_cmd) - C_f;

        % Swing foot zero force constraints (unchanged)

        A_swing_const = []; b_swing_const = [];

        for i = 1:4
            if contact_state(i) == 0
                f_idx_start = 6 + (i-1)*3 + 1;
                A_sub = zeros(3, 18); A_sub(1:3, f_idx_start:f_idx_start+2) = eye(3);
                A_swing_const = [A_swing_const; A_sub]; b_swing_const = [b_swing_const; 0; 0; 0];
            end
        end

        A_eq = [A_dyn; A_swing_const]; b_eq = [b_dyn; b_swing_const];

        % FIX 2: Friction cone ONLY for stance legs

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