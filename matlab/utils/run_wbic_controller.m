function [tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd, f_r_final] = run_wbic_controller(state, params)

    tau_j = zeros(12,1);
    q_j_cmd = state.qj_pos;
    q_j_vel_cmd = zeros(12,1);
    contact_state = zeros(4,1);
    f_r_final = zeros(12,1);  % Initialize final forces output
    
    % Debug Counter
    persistent wbic_cnt;
    if isempty(wbic_cnt), wbic_cnt = 0; end
    wbic_cnt = wbic_cnt + 1;
    do_print = (mod(wbic_cnt, 20) == 0); % Print every 0.5s

    % Persistent state for event-based contact detection
    persistent prev_contact_state;
    if isempty(prev_contact_state)
        prev_contact_state = ones(4, 1);  % Start in stance
    end

    try
        %% --- 1. Setup Dynamics ---
        q_pos_curr = state.qj_pos;
        H = reshape(state.inertia_mat, [18, 18]); 
        C = state.bias_force;                     
        J_c = reshape(state.J_gc, [12, 18]);      
        
        H_f = H(1:6, :); H_ff = H(1:6, 1:6); H_j = H(7:18, :);      
        C_f = C(1:6); C_j = C(7:18);        
        JcT_f = J_c(:, 1:6)'; JcT_j = J_c(:, 7:18)';  

        %% --- 2. Setup MPC Plan (MOCK / STANDALONE) ---

        if ~isjava(params.mpc_plan) || isempty(params.mpc_plan)
            MASS = 12.45; GRAVITY = 9.81;
            mock_plan = struct();
            mock_plan.contact = [1;1;1;1];

            % Calculate center of support
            feet_x = state.p_gc([1, 4, 7, 10]);
            feet_y = state.p_gc([2, 5, 8, 11]);
            center_x = mean(feet_x);
            center_y = mean(feet_y);

            % Target: Centered over feet at specific height
            height = 0.35;
            mock_plan.body_pos_cmd = [center_x; center_y; height];
            mock_plan.body_rpy_cmd = [0;0;0];
            mock_plan.body_vel_cmd = zeros(3,1);
            mock_plan.body_omega_cmd = zeros(3,1);
            mock_plan.foot_pos_cmd = state.p_gc;
            mock_plan.foot_vel_cmd = zeros(12,1);

            % --- Compute reaction forces using QP (like simplified MPC) ---
            % This distributes forces to achieve equilibrium + orientation correction
            p_com = state.position;
            p_feet = reshape(state.p_gc, [3, 4]);

            % Build force distribution matrix
            % [sum of forces = mg] and [sum of moments = desired moment]
            A_fd = zeros(6, 12);
            for i = 1:4
                idx = (i-1)*3 + (1:3);
                A_fd(1:3, idx) = eye(3);  % Force sum
                r_i = p_feet(:, i) - p_com;
                A_fd(4:6, idx) = skew_matrix(r_i);  % Moment sum
            end

            % Desired wrench: gravity compensation + orientation correction
            % Add moment to correct pitch/roll errors (negative feedback)
            kp_ori = 50;  % Orientation correction gain
            % Negative sign: if pitched forward (+), apply backward moment (-)
            desired_moment = -kp_ori * [state.rpy(1); state.rpy(2); 0];
            b_fd = [0; 0; MASS * GRAVITY; desired_moment];

            % QP to find forces: min ||f||^2 s.t. A*f = b, friction cone
            H_fd = eye(12);
            f_fd = zeros(12, 1);

            % Friction cone constraints
            mu = 0.6;
            W_leg = [-1 0 mu; 1 0 mu; 0 -1 mu; 0 1 mu; 0 0 1];
            A_ineq_fd = -blkdiag(W_leg, W_leg, W_leg, W_leg);
            b_ineq_fd = zeros(20, 1);

            opts = optimoptions('quadprog', 'Display', 'off');
            [f_sol, ~, exitflag] = quadprog(H_fd, f_fd, A_ineq_fd, b_ineq_fd, A_fd, b_fd, [], [], [], opts);

            if exitflag == 1
                mock_plan.reaction_force = f_sol;
            else
                % Fallback to equal forces
                f_z = (MASS * GRAVITY) / 4;
                mock_plan.reaction_force = repmat([0; 0; f_z], 4, 1);
                if do_print
                    fprintf('[WBIC Standalone] Force distribution QP failed, using equal forces\n');
                end
            end

            params.mpc_plan = mock_plan;
        end
        
        f_r_mpc = params.mpc_plan.reaction_force;
        contact_cmd = params.mpc_plan.contact;

        p_gc_curr = reshape(state.p_gc, [3, 4]);

        %% --- 2b. Event-Based Contact Detection (Bledt et al. 2018) ---
        % Use force feedback to detect early touch-down during swing
        force_threshold = 20.0;  % N - threshold for touch-down detection
        contact_state = zeros(4, 1);

        for leg = 1:4
            % Extract foot force magnitude from reaction forces
            idx = 3*leg-2:3*leg;
            foot_force = norm(f_r_mpc(idx));

            % Event-based override: if MPC commands swing BUT force detected -> switch to stance
            if contact_cmd(leg) == 0 && foot_force > force_threshold
                % Early touch-down detected!
                contact_state(leg) = 1;
            else
                % Use MPC contact command
                contact_state(leg) = contact_cmd(leg);
            end
        end

        % Update previous contact state
        prev_contact_state = contact_state;

        % Use trajectories from MPC (already generated with Bézier/stance modulation)
        p_gc_des = reshape(params.mpc_plan.foot_pos_cmd, [3, 4]);

        try
            v_gc_des = reshape(params.mpc_plan.foot_vel_cmd, [3, 4]);
        catch
            v_gc_des = zeros(3, 4);
        end
        
        q_dot_full = [state.velocity; state.omega; state.qj_vel];
        v_gc_act = J_c * q_dot_full; 

        %% --- 3. WEIGHTED SUM TASK CONTROL ---
        % All tasks are combined using a fixed weighted sum approach
        % (replaces null-space projection method)
        % Task priorities: Stance >> Orientation > Position > Swing
        
        % Paper Table I gains
        kp_base = 100; kd_base = 10;

        % Gain scheduling: Use different gains for standing vs. locomotion
        % Standing: Lower gains to avoid oscillations and noise amplification
        % Locomotion: Higher gains (Hyun et al. 2014) for aggressive tracking
        n_legs_in_contact = sum(contact_state);
        if n_legs_in_contact == 4
            % All feet in stance = standing mode
            kp_foot = 100; kd_foot = 10;  % Original stable gains
            gait_mode_str = 'STAND';
        else
            % Dynamic gait (trot, etc)
            kp_foot = 450; kd_foot = 50;  % Hyun et al. (2014) aggressive gains
            gait_mode_str = 'TROT';
        end 

        % --- TASK 0: Stance Constraints ---
        J_stance = [];
        for i = 1:4
            if contact_state(i) == 1, J_stance = [J_stance; J_c(3*i-2 : 3*i, :)]; end
        end
        
        if ~isempty(J_stance)
            x_ddot_0 = zeros(size(J_stance, 1), 1);  % Zero acceleration for stance feet
        else
            J_stance = zeros(0, 18);  % Empty matrix if no stance legs
            x_ddot_0 = zeros(0, 1);
        end

        % --- TASK 1: Body Orientation ---
        J_1 = [zeros(3,3), eye(3), zeros(3,12)]; 
        rot_err = params.mpc_plan.body_rpy_cmd - state.rpy;
        omega_err = params.mpc_plan.body_omega_cmd - state.omega;
        x_ddot_1 = 0 * kp_base * rot_err + 0 * kd_base * omega_err;
        % fprintf("nuking body orientation");
        
        % Store for debug
        ori_err_deg = rot_err * 180/pi;

        % --- TASK 2: Body Position ---
        J_2 = [eye(3), zeros(3,3), zeros(3,12)]; 
        pos_err = params.mpc_plan.body_pos_cmd - state.position;
        vel_err = params.mpc_plan.body_vel_cmd - state.velocity;
        x_ddot_2 = kp_base * pos_err + kd_base * vel_err;

        % Store for debug
        pos_err_m = pos_err;

        % --- TASK 3: Swing Foot ---
        J_swing = []; x_ddot_3 = [];
        for i = 1:4
            if contact_state(i) == 0
                J_swing = [J_swing; J_c(3*i-2 : 3*i, :)];
                idx = 3*i-2 : 3*i;
                p_err = p_gc_des(:, i) - p_gc_curr(:, i);
                v_err = v_gc_des(:, i) - v_gc_act(idx);
                acc_swing = kp_foot * p_err + kd_foot * v_err;
                x_ddot_3 = [x_ddot_3; acc_swing];
            end
        end
        
        if isempty(J_swing)
            J_swing = zeros(0, 18);  % Empty matrix if no swing legs
            x_ddot_3 = zeros(0, 1);
        end

        % --- WEIGHTED SUM SOLUTION ---
        % Stack all tasks: J_all * q_ddot = x_ddot_all
        J_all = [J_stance; J_1; J_2; J_swing];
        x_ddot_all = [x_ddot_0; x_ddot_1; x_ddot_2; x_ddot_3];
        
        % Priority weights: higher weight = higher priority
        % Stance gets very high weight (almost hard constraint)
        % Orientation > Position > Swing
        w_stance = 1e6;   % Very high priority for stance constraints
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
        
        % Solve weighted least squares: min ||W^(1/2) * (J_all * q_ddot - x_ddot_all)||^2
        % Using regularization for numerical stability
        reg = 1e-6;  % Small regularization term
        q_ddot_cmd = (J_all' * W * J_all + reg * eye(18)) \ (J_all' * W * x_ddot_all);

        

        % Joint Integration
        dt_wbic = 0.002; 
        q_j_acc = q_ddot_cmd(7:18);
        q_j_vel_cmd = state.qj_vel + q_j_acc * dt_wbic;
        q_j_cmd = state.qj_pos + q_j_vel_cmd * dt_wbic;

        %% --- 4. QP SOLVER ---
        n_vars = 18;
        Q1 = 1.0 * eye(12); Q2 = 0.1 * eye(6);
        H_qp = 2 * blkdiag(Q2, Q1); f_qp = zeros(n_vars, 1);

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

        % Solve QP
        options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
        [x_sol, ~, flag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);

        if flag == 1
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