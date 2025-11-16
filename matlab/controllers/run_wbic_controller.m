function [tau_j, contact_state, params, q_j_cmd, q_j_vel_cmd] = run_wbic_controller(state, params)
    
    % Initialize output arguments to safe values
    tau_j = zeros(12,1);
    q_j_cmd = state.qj_pos;     % Default to current pose on init/fail
    q_j_vel_cmd = zeros(12,1);  % Default to zero velocity
    contact_state = zeros(4,1);
    
    try
        %% --- 1. Check for MPC Plan ---
        if ~isjava(params.mpc_plan)
            fprintf('No MPC plan received yet. Commanding zero torques.\n');
        
        else
            %% --- 2. Get State & Dynamics ---
            f_r_mpc = params.mpc_plan.reaction_force;
            contact_cmd = params.mpc_plan.contact;
            contact_state = contact_cmd; 
            q_pos_curr = state.qj_pos;
            q_vel_curr = state.qj_vel;
            H = reshape(state.inertia_mat, [18, 18]); 
            C = state.bias_force;                     
            J_c = reshape(state.J_gc, [12, 18]);      
            H_f = H(1:6, :);       
            H_ff = H(1:6, 1:6);    
            H_j = H(7:18, :);      
            C_f = C(1:6);         
            C_j = C(7:18);        
            JcT_f = J_c(:, 1:6)';   
            JcT_j = J_c(:, 7:18)';  
            J_task_contact = J_c;
            J_task_body = [eye(6), zeros(6, 12)];
            J_task_joint = [zeros(12, 6), eye(12)];

            %% --- 3. KINEMATIC Hierarchies (Eq 16 & 17) ---
            
            % --- 3a. Position Hierarchy (Eq. 16) ---
            delta_q_prev = zeros(18, 1);
            N_prev_pos = eye(18);
            % Task 1 (Contact)
            J_pre_1_pos = J_task_contact * N_prev_pos;
            J_pinv_1_pos = pinv(J_pre_1_pos);
            e_1_pos = zeros(12, 1);
            delta_q_1 = delta_q_prev + J_pinv_1_pos * (e_1_pos - J_pre_1_pos * delta_q_prev);
            N_1_pos = N_prev_pos * (eye(18) - J_pinv_1_pos * J_pre_1_pos);
            delta_q_prev = delta_q_1;
            N_prev_pos = N_1_pos;
            
            % Task 2 (Body)
            J_pre_2_pos = J_task_body * N_prev_pos;
            J_pinv_2_pos = pinv(J_pre_2_pos);
            
            rpy_des_kin = params.mpc_plan.body_rpy_cmd;
            
            e_2_pos = [params.mpc_plan.body_pos_cmd - state.position; 
                       rpy_des_kin - state.rpy];
            
            delta_q_2 = delta_q_prev + J_pinv_2_pos * (e_2_pos - J_pre_2_pos * delta_q_prev);
            N_2_pos = N_prev_pos * (eye(18) - J_pinv_2_pos * J_pre_2_pos);
            delta_q_prev = delta_q_2;
            N_prev_pos = N_2_pos;

            %{

            % Task 3 (Joint)
            J_pre_3_pos = J_task_joint * N_prev_pos;
            J_pinv_3_pos = pinv(J_pre_3_pos);
            e_3_pos = zeros(12, 1); % Task is disabled
            delta_q_3 = delta_q_prev + J_pinv_3_pos * (e_3_pos - J_pre_3_pos * delta_q_prev);
            delta_q_j = delta_q_3(7:18);
            q_j_cmd = q_pos_curr + delta_q_j;

            %}

            % --- 3b. Velocity Hierarchy (Eq. 17) ---
            q_dot_cmd_prev = zeros(18, 1);
            N_prev_vel = eye(18);

            % Task 1 (Contact)
            J_pre_1_vel = J_task_contact * N_prev_vel;
            J_pinv_1_vel = pinv(J_pre_1_vel);
            x_dot_des_1 = zeros(12, 1);
            q_dot_cmd_1 = q_dot_cmd_prev + J_pinv_1_vel * (x_dot_des_1 - J_pre_1_vel * q_dot_cmd_prev);
            N_1_vel = N_prev_vel * (eye(18) - J_pinv_1_vel * J_pre_1_vel);
            q_dot_cmd_prev = q_dot_cmd_1;
            N_prev_vel = N_1_vel;

            % Task 2 (Body)
            J_pre_2_vel = J_task_body * N_prev_vel;
            J_pinv_2_vel = pinv(J_pre_2_vel);
            x_dot_des_2 = [params.mpc_plan.body_vel_cmd; params.mpc_plan.body_omega_cmd];
            q_dot_cmd_2 = q_dot_cmd_prev + J_pinv_2_vel * (x_dot_des_2 - J_pre_2_vel * q_dot_cmd_prev);
            N_2_vel = N_prev_vel * (eye(18) - J_pinv_2_vel * J_pre_2_vel);
            q_dot_cmd_prev = q_dot_cmd_2;
            N_prev_vel = N_2_vel;


            %{

            % Task 3 (Joint)
            J_pre_3_vel = J_task_joint * N_prev_vel;
            J_pinv_3_vel = pinv(J_pre_3_vel);
            x_dot_des_3 = params.q_vel_des;
            q_dot_cmd_3 = q_dot_cmd_prev + J_pinv_3_vel * (x_dot_des_3 - J_pre_3_vel * q_dot_cmd_prev);
            q_j_vel_cmd = q_dot_cmd_3(7:18);

            %}

            %% --- 4. ACCELERATION Hierarchy (Eq 18) ---
            q_ddot_cmd_prev = zeros(18, 1);
            N_prev = eye(18);

            % --- TASK 1: Stance Foot Contact ---
            J_pre_1 = J_task_contact * N_prev;
            J_bar_dyn_1 = dynamic_pinv(J_pre_1, H);
            x_ddot_cmd_task1 = zeros(12, 1);
            q_ddot_cmd_1 = q_ddot_cmd_prev + J_bar_dyn_1 * (x_ddot_cmd_task1 - J_pre_1 * q_ddot_cmd_prev);
            J_pinv_1 = pinv(J_pre_1); 
            N_1 = N_prev * (eye(18) - J_pinv_1 * J_pre_1);
            q_ddot_cmd_prev = q_ddot_cmd_1;
            N_prev = N_1;

            % --- TASK 2: Body Pose (Eq. 22) ---
            J_pre_2 = J_task_body * N_prev;
            J_bar_dyn_2 = dynamic_pinv(J_pre_2, H);
            kp_pos = 250; kd_pos = 25;
            kp_rot = 250; kd_rot = 25;
            pos_des = params.mpc_plan.body_pos_cmd; 
            vel_des = params.mpc_plan.body_vel_cmd;
            acc_pos_des = kp_pos * (pos_des - state.position) + kd_pos * (vel_des - state.velocity);
            
            rpy_des_dyn = params.mpc_plan.body_rpy_cmd; 
            
            omega_des = params.mpc_plan.body_omega_cmd;
            acc_rpy_des = kp_rot * (rpy_des_dyn - state.rpy) + kd_rot * (omega_des - state.omega);
            x_ddot_cmd_task2 = [acc_pos_des; acc_rpy_des];
            q_ddot_cmd_2 = q_ddot_cmd_prev + J_bar_dyn_2 * (x_ddot_cmd_task2 - J_pre_2 * q_ddot_cmd_prev);
            J_pinv_2 = pinv(J_pre_2); 
            N_2 = N_prev * (eye(18) - J_pinv_2 * J_pre_2);
            q_ddot_cmd_prev = q_ddot_cmd_2;
            N_prev = N_2;

            q_ddot_cmd = q_ddot_cmd_prev;
            
            %{

            % --- TASK 3: Joint Posture (Eq. 22) ---
            J_pre_3 = J_task_joint * N_prev;
            J_bar_dyn_3 = dynamic_pinv(J_pre_3, H);
            kp_task3 = 0.0; % Keep this off
            kd_task3 = 0.0;
            KP_vec_task3 = ones(12, 1) * kp_task3;
            KD_vec_task3 = ones(12, 1) * kd_task3;
            pos_error_joints = params.q_des_base - q_pos_curr;
            vel_error_joints = params.q_vel_des - q_vel_curr;
            x_ddot_cmd_task3 = KP_vec_task3 .* pos_error_joints + KD_vec_task3 .* vel_error_joints;
            q_ddot_cmd_3 = q_ddot_cmd_prev + J_bar_dyn_3 * (x_ddot_cmd_task3 - J_pre_3 * q_ddot_cmd_prev);
            q_ddot_cmd = q_ddot_cmd_3;

            %}

            %% --- 5. STEP 3: Hybrid QP Solver (Eq. 25) ---
            w_a_f = 10.0;  % Prioritize acceleration
            w_f_r = 0.001; % Relax forces
            Q_a_f = w_a_f * eye(6);
            Q_f_r = w_f_r * eye(12);
            H_qp = 2 * blkdiag(Q_a_f, Q_f_r);
            f_qp = zeros(18, 1);
            A_eq = [H_ff, -JcT_f];
            b_eq = (JcT_f * f_r_mpc) - (H_f * q_ddot_cmd) - C_f;
            mu = 0.7; 
            W_foot = [ -1, 0, mu; 1, 0, mu; 0,-1, mu; 0, 1, mu; 0, 0,  1 ];
            W = blkdiag(W_foot, W_foot, W_foot, W_foot);
            A_ineq = [zeros(20, 6), -W];
            b_ineq = W * f_r_mpc;

            %% --- 6. Solve QP & Final Torque (Eq. 26) ---
            options = optimoptions('quadprog', 'Display', 'off');
            [solution, fval, exitflag] = quadprog(H_qp, f_qp, ...
                                            A_ineq, b_ineq, ...
                                            A_eq, b_eq, ...
                                            [], [], [], options);
            
            if exitflag == 1
                % --- QP Success ---
                delta_a_f = solution(1:6);
                delta_f_r = solution(7:end);
                q_ddot_final = q_ddot_cmd;
                q_ddot_final(1:6) = q_ddot_final(1:6) + delta_a_f;
                f_r_final = f_r_mpc + delta_f_r;
                tau_j = H_j * q_ddot_final + C_j - JcT_j * f_r_final;
            
            else
                % --- QP Failure ---
                disp('QP FAILED TO SOLVE! Commanding zero torques.');
            end
        end
        
    catch ME
        % --- This block catches any error from the 'try' block ---
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
        disp('--- AN ERROR OCCURRED IN WBC CONTROLLER ---');
        disp(ME.message);
        fprintf('Error occurred in %s on line %d\n', ME.stack(1).name, ME.stack(1).line);
    end
    
end