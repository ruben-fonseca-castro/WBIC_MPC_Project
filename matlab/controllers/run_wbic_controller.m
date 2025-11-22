function [tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd, f_r_final] = run_wbic_controller(state, params)

    tau_j = zeros(12,1);
    q_j_cmd = state.qj_pos;
    q_j_vel_cmd = zeros(12,1);
    contact_state = zeros(4,1);
    f_r_final = zeros(12,1);  % Initialize final forces output

    %% ==================== TASK GAINS (TUNE HERE) ====================
    % Task 1: Body Orientation (Roll, Pitch, Yaw tracking)
    kp_ori = 100;   % [rad/s^2 per rad] Orientation stiffness
    kd_ori = 10;    % [rad/s^2 per rad/s] Orientation damping

    % Task 2: Body Position (X, Y, Z tracking)
    kp_pos = 100;   % [m/s^2 per m] Position stiffness
    kd_pos = 10;    % [m/s^2 per m/s] Position damping

    % Task 3: Swing Foot (per-foot position tracking)
    % These are gain-scheduled based on gait mode (set below in code)
    kp_swing_stand = 100;   % Standing mode - lower to avoid oscillations
    kd_swing_stand = 10;
    kp_swing_trot = 450;    % Trotting mode - Hyun et al. (2014) aggressive gains
    kd_swing_trot = 50;
    %% ================================================================

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

        %% --- 2. Read MPC Plan ---
        % NOTE: Mock MPC failsafe removed to avoid confounding WBIC tests.
        % WBIC now requires a valid MPC plan to be published.
        if ~isjava(params.mpc_plan) || isempty(params.mpc_plan)
            % No MPC plan received - return zero torques
            tau_j = zeros(12, 1);
            contact_state = [0; 0; 0; 0];
            q_j_cmd = state.qj_pos;
            q_j_vel_cmd = zeros(12, 1);
            f_r_final = zeros(12, 1);
            if do_print
                fprintf('[WBIC] No MPC plan received, outputting zero torques\n');
            end
            return;
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

        %% --- 3. KINEMATIC HIERARCHY ---
        % Task 0: Stance Constraints (Hard)
        % Task 1: Body Orientation (Highest Task)
        % Task 2: Body Position
        % Task 3: Swing Foot Position (Lowest)

        q_ddot_prev = zeros(18, 1);
        N_prev = eye(18);

        % Gain scheduling for swing foot: standing vs locomotion
        n_legs_in_contact = sum(contact_state);
        if n_legs_in_contact == 4
            kp_swing = kp_swing_stand;
            kd_swing = kd_swing_stand;
            gait_mode_str = 'STAND';
        else
            kp_swing = kp_swing_trot;
            kd_swing = kd_swing_trot;
            gait_mode_str = 'TROT';
        end 

        % --- TASK 0: Stance Constraints ---
        J_stance = [];
        for i = 1:4
            if contact_state(i) == 1, J_stance = [J_stance; J_c(3*i-2 : 3*i, :)]; end
        end
        
        if ~isempty(J_stance)
            x_ddot_0 = zeros(size(J_stance, 1), 1); 
            J_pre_0 = J_stance; J_bar_0 = dynamic_pinv(J_pre_0, H);
            q_ddot_0 = J_bar_0 * x_ddot_0;
            N_prev = eye(18) - J_bar_0 * J_pre_0; 
            q_ddot_prev = q_ddot_0;
        end

        % --- TASK 1: Body Orientation ---
        J_1 = [zeros(3,3), eye(3), zeros(3,12)];
        rot_err = params.mpc_plan.body_rpy_cmd - state.rpy;
        omega_err = params.mpc_plan.body_omega_cmd - state.omega;
        x_ddot_1 = kp_ori * rot_err + kd_ori * omega_err;
        
        J_pre_1 = J_1 * N_prev; J_bar_1 = dynamic_pinv(J_pre_1, H);
        q_ddot_1 = q_ddot_prev + J_bar_1 * (x_ddot_1 - J_pre_1 * q_ddot_prev);
        J_pinv_1 = pinv(J_pre_1); N_prev = N_prev * (eye(18) - J_pinv_1 * J_pre_1);
        q_ddot_prev = q_ddot_1; 
        
        % Store for debug
        ori_err_deg = rot_err * 180/pi;

        % --- TASK 2: Body Position ---
        J_2 = [eye(3), zeros(3,3), zeros(3,12)];
        pos_err = params.mpc_plan.body_pos_cmd - state.position;
        vel_err = params.mpc_plan.body_vel_cmd - state.velocity;
        x_ddot_2 = kp_pos * pos_err + kd_pos * vel_err;
        
        J_pre_2 = J_2 * N_prev; J_bar_2 = dynamic_pinv(J_pre_2, H);
        q_ddot_2 = q_ddot_prev + J_bar_2 * (x_ddot_2 - J_pre_2 * q_ddot_prev);
        J_pinv_2 = pinv(J_pre_2); N_prev = N_prev * (eye(18) - J_pinv_2 * J_pre_2);
        q_ddot_prev = q_ddot_2;

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
                acc_swing = kp_swing * p_err + kd_swing * v_err;
                x_ddot_3 = [x_ddot_3; acc_swing];
            end
        end
        
        if ~isempty(J_swing)
            J_pre_3 = J_swing * N_prev; J_bar_3 = dynamic_pinv(J_pre_3, H);
            q_ddot_3 = q_ddot_prev + J_bar_3 * (x_ddot_3 - J_pre_3 * q_ddot_prev);
            q_ddot_cmd = q_ddot_3;
        else
            q_ddot_cmd = q_ddot_prev;
        end

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
            fprintf('Mode: %s | QP flag=%d %s\n', gait_mode_str, flag, wbic_qp_status(flag));
            fprintf('Gains: Ori[kp=%d,kd=%d] Pos[kp=%d,kd=%d] Swing[kp=%d,kd=%d]\n', ...
                kp_ori, kd_ori, kp_pos, kd_pos, kp_swing, kd_swing);
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