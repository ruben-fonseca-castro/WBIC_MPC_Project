function [tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd] = run_wbic_controller(state, params)
    
    tau_j = zeros(12,1);
    q_j_cmd = state.qj_pos;
    q_j_vel_cmd = zeros(12,1);
    contact_state = zeros(4,1);
    
    try
        q_pos_curr = state.qj_pos;
        H = reshape(state.inertia_mat, [18, 18]); 
        C = state.bias_force;                     
        J_c = reshape(state.J_gc, [12, 18]);      
        
        H_f = H(1:6, :); H_ff = H(1:6, 1:6); H_j = H(7:18, :);      
        C_f = C(1:6); C_j = C(7:18);        
        JcT_f = J_c(:, 1:6)'; JcT_j = J_c(:, 7:18)';  

        if ~isjava(params.mpc_plan) || isempty(params.mpc_plan)
            MASS = 12.45; GRAVITY = 9.81; f_z = (MASS*GRAVITY)/4;
            mock_plan = struct();
            mock_plan.contact = [1;1;1;1];
            mock_plan.reaction_force = repmat([0;0;f_z],4,1);
            mock_plan.body_pos_cmd = [state.position(1); state.position(2); 0.28]; 
            mock_plan.body_rpy_cmd = [0;0;0]; 
            mock_plan.body_vel_cmd = zeros(3,1); mock_plan.body_omega_cmd = zeros(3,1);
            mock_plan.foot_pos_cmd = state.p_gc; 
            mock_plan.foot_vel_cmd = zeros(12,1);
            params.mpc_plan = mock_plan;
        end
        
        f_r_mpc = params.mpc_plan.reaction_force;
        contact_cmd = params.mpc_plan.contact;
        contact_state = contact_cmd; 
        
        J_task_body = [eye(6), zeros(6, 12)];
        
        p_gc_curr = reshape(state.p_gc, [3, 4]); 
        p_gc_des = reshape(params.mpc_plan.foot_pos_cmd, [3, 4]); 
        
        try
            v_gc_des = reshape(params.mpc_plan.foot_vel_cmd, [3, 4]);
        catch
            v_gc_des = zeros(3, 4);
        end
        
        %% --- Kinematics ---
        delta_q_prev = zeros(18, 1); N_prev = eye(18);
        
        % Task 1: Feet
        J_1 = []; e_1 = [];
        for i = 1:4 
            J_leg = J_c(3*i-2 : 3*i, :);
            if contact_cmd(i) == 1, e_leg = zeros(3, 1); 
            else, e_leg = p_gc_des(:, i) - p_gc_curr(:, i); end
            J_1 = [J_1; J_leg]; e_1 = [e_1; e_leg];
        end
        J_pre_1 = J_1 * N_prev; J_pinv_1 = pinv(J_pre_1);
        delta_q_1 = delta_q_prev + J_pinv_1 * (e_1 - J_pre_1 * delta_q_prev);
        N_1 = N_prev * (eye(18) - J_pinv_1 * J_pre_1);
        delta_q_prev = delta_q_1; N_prev = N_1;
        
        % Task 2: Body
        J_pre_2 = J_task_body * N_prev; J_pinv_2 = pinv(J_pre_2);
        e_2 = [params.mpc_plan.body_pos_cmd - state.position; params.mpc_plan.body_rpy_cmd - state.rpy];
        delta_q_2 = delta_q_prev + J_pinv_2 * (e_2 - J_pre_2 * delta_q_prev);
        
        q_j_cmd = q_pos_curr + delta_q_2(7:18);

        %% --- Acceleration (Dynamics) ---
        q_ddot_prev = zeros(18, 1); N_prev = eye(18);
        q_dot_full = [state.velocity; state.omega; state.qj_vel];
        v_gc_act = J_c * q_dot_full; 

        % Task 1: Feet
        x_ddot_1 = zeros(12, 1);
        kp_swing = 800; kd_swing = 20; 
        
        for i = 1:4
            idx = 3*i-2 : 3*i;
            if contact_cmd(i) == 1
                x_ddot_1(idx) = zeros(3,1);
            else
                pos_err = p_gc_des(:, i) - p_gc_curr(:, i);
                vel_err = v_gc_des(:, i) - v_gc_act(idx); 
                x_ddot_1(idx) = kp_swing * pos_err + kd_swing * vel_err;
            end
        end
        
        J_pre_1 = J_c * N_prev; J_bar_1 = dynamic_pinv(J_pre_1, H);
        q_ddot_1 = q_ddot_prev + J_bar_1 * (x_ddot_1 - J_pre_1 * q_ddot_prev);
        J_pinv_1 = pinv(J_pre_1); N_1 = N_prev * (eye(18) - J_pinv_1 * J_pre_1);
        q_ddot_prev = q_ddot_1; N_prev = N_1;

        % --- Task 2: Body (VECTOR GAINS) ---
        % FIX: Increase Pitch stiffness (kp_rot(2)) and Vertical stiffness (kp_pos(3))
        kp_pos_vec = [200; 200; 800]; % Increase Z stiffness to 800
        kd_pos_vec = [20; 20; 50];    % Increase Z damping
        
        % Increase Pitch (Y-axis rotation) stiffness significantly
        kp_rot_vec = [400; 800; 400]; % Roll, Pitch, Yaw
        kd_rot_vec = [30; 50; 30];
        
        pos_err = params.mpc_plan.body_pos_cmd - state.position;
        vel_err = params.mpc_plan.body_vel_cmd - state.velocity;
        
        % Element-wise multiplication (.*) for vector gains
        acc_pos = kp_pos_vec .* pos_err + kd_pos_vec .* vel_err;
        
        rot_err = params.mpc_plan.body_rpy_cmd - state.rpy;
        omega_err = params.mpc_plan.body_omega_cmd - state.omega;
        
        % Use vector gains for rotation too
        acc_rot = kp_rot_vec .* rot_err + kd_rot_vec .* omega_err;
        
        x_ddot_2 = [acc_pos; acc_rot];
        
        J_pre_2 = J_task_body * N_prev; J_bar_2 = dynamic_pinv(J_pre_2, H);
        q_ddot_2 = q_ddot_prev + J_bar_2 * (x_ddot_2 - J_pre_2 * q_ddot_prev);
        q_ddot_cmd = q_ddot_2;

        %% --- QP Solver ---
        w_a = 10.0; w_f = 0.001;
        H_qp = 2 * blkdiag(w_a*eye(6), w_f*eye(12)); f_qp = zeros(18,1);
        A_eq = [H_ff, -JcT_f]; b_eq = (JcT_f * f_r_mpc) - (H_f * q_ddot_cmd) - C_f;
        mu = 0.6; 
        W_foot = [ -1, 0, mu; 1, 0, mu; 0,-1, mu; 0, 1, mu; 0, 0,  1 ];
        W = blkdiag(W_foot, W_foot, W_foot, W_foot);
        A_ineq = [zeros(20, 6), -W]; b_ineq = W * f_r_mpc;
        
        options = optimoptions('quadprog', 'Display', 'off');
        [sol, ~, flag] = quadprog(H_qp, f_qp, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
        
        if flag == 1
            q_ddot_final = q_ddot_cmd;
            q_ddot_final(1:6) = q_ddot_final(1:6) + sol(1:6);
            f_r_final = f_r_mpc + sol(7:end);
            tau_j = H_j * q_ddot_final + C_j - JcT_j * f_r_final;
        else
            tau_j = zeros(12,1);
        end
    catch ME
         disp(ME.message);
    end
end