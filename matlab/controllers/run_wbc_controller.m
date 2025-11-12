function [tau_cmd, contact_state, params] = run_wbc_controller(state, params)
    % This is the "Whole-Body Impulse Controller" (WBC) brain.
    % It minimizes a weighted sum of tasks:
    % 1. (HIGH) Stance Foot: Keep feet from accelerating.
    % 2. (HIGH) Pose Task: Track the desired body/joint acceleration.
    % 3. (LOW) Force Task: Track the desired forces from the MPC.
    
    %% --- 1. Check for MPC Plan ---
    if ~isfield(params.mpc_plan, 'reaction_force')
        tau_cmd = zeros(12,1);
        contact_state = [0; 0; 0; 0];
        return;
    end
    
    %% --- 2. Get Commands & State ---
    Fc_desired = params.mpc_plan.reaction_force;
    contact_cmd = params.mpc_plan.contact;
    q_pos_curr = state.qj_pos;
    q_vel_curr = state.qj_vel;

    %% --- 3. Handle Setpoint Ramp ---
    if ~params.is_initialized
        params.q_des_ramping = q_pos_curr;
        params.is_initialized = true;
    end
    
    % Get final target (with joystick "lean")
    q_des_final = params.q_des_base;
    pitch_offset = params.joy_state.left_stick_y * 0.5;
    q_des_final([2, 5, 8, 11]) = params.q_des_base([2, 5, 8, 11]) + pitch_offset;
    roll_offset = params.joy_state.right_stick_x * 0.3;
    q_des_final([1, 7]) = params.q_des_base([1, 7]) + roll_offset;
    q_des_final([4, 10]) = params.q_des_base([4, 10]) - roll_offset;
    
    % Update the ramp
    ramp_error = q_des_final - params.q_des_ramping;
    step = max(min(ramp_error, params.max_step_size), -params.max_step_size);
    params.q_des_ramping = params.q_des_ramping + step;
    
    %% --- 4. Get Dynamics "Ingredients" ---
    H_matrix = reshape(state.inertia_mat, [18, 18]);
    C_vector = state.bias_force;
    Jc = reshape(state.J_gc, [12, 18]);
    Jc_dot_q_dot = state.dJdq_gc;
    
    H_base = H_matrix(1:6, :);
    C_base = C_vector(1:6);
    H_joint = H_matrix(7:18, :);
    C_joint = C_vector(7:18);
    Jc_T_base = Jc(:, 1:6)';
    Jc_T_joint = Jc(:, 7:18)';
    
    %% --- 5. Define Tasks & Weights ---
    
    % Task 1: Stance Foot (High Priority)
    % Goal: Jc*q_ddot + Jc_dot*q_dot = 0
    % This means Jc*q_ddot = -Jc_dot_q_dot
    w_stance = 1.0;
    A_stance = [Jc, zeros(12, 12)]; % (12x30)
    b_stance = -Jc_dot_q_dot;       % (12x1)

    % Task 2: Pose (High Priority)
    % Goal: q_ddot = q_ddot_des
    pos_error = params.q_des_ramping - q_pos_curr;
    vel_error = params.q_vel_des - q_vel_curr;
    q_j_ddot_des = params.KP_vec .* pos_error + params.KD_vec .* vel_error;
    q_ddot_des = [zeros(6,1); q_j_ddot_des];
    
    w_pose = 1.0;
    A_pose = [eye(18), zeros(18, 12)]; % (18x30)
    b_pose = q_ddot_des;              % (18x1)
    
    % Task 3: Force (Low Priority, for now)
    % Goal: Fc = Fc_desired
    w_force = 0.001;
    A_force = [zeros(12, 18), eye(12)]; % (12x30)
    b_force = Fc_desired;              % (12x1)

    %% --- 6. Setup the WBC Quadratic Program ---
    % We are solving for x = [q_ddot (18x1); Fc (12x1)]
    % The objective is to minimize the sum of weighted task errors:
    % min: w_s * ||A_s*x - b_s||^2 + w_p * ||A_p*x - b_p||^2 + w_f * ||A_f*x - b_f||^2
    %
    % This is a standard QP: min 0.5*x'*H_qp*x + f_qp'*x
    
    % H_qp = 2 * (w_s*A_s'*A_s + w_p*A_p'*A_p + w_f*A_f'*A_f)
    H_qp = 2 * (w_stance * (A_stance' * A_stance) + ...
                w_pose   * (A_pose' * A_pose)     + ...
                w_force  * (A_force' * A_force));
                
    % f_qp = -2 * (w_s*A_s'*b_s + w_p*A_p'*b_p + w_f*A_f'*b_f)
    f_qp = -2 * (w_stance * (A_stance' * b_stance) + ...
                w_pose   * (A_pose' * b_pose)     + ...
                w_force  * (A_force' * b_force));

    % --- Equality Constraints (Physics) ---
    % This is the *only* hard constraint: the laws of physics.
    % H_base*q_ddot - Jc_T_base*Fc = -C_base
    A_eq = [ H_base, -Jc_T_base ]; % (6x30)
    b_eq = -C_base;                % (6x1)

    %% --- 7. Solve Quadratic Program ---
    options = optimoptions('quadprog', 'Display', 'off');
    
    % We will add friction constraints (A_ineq, b_ineq) later
    [solution, fval, exitflag] = quadprog(H_qp, f_qp, [], [], A_eq, b_eq, [], [], [], options);
    
    if exitflag ~= 1
        disp('QP FAILED TO SOLVE! Commanding zero torques.');
        tau_cmd = zeros(12,1);
        contact_state = [0; 0; 0; 0];
        return;
    end
    
    q_ddot_solution = solution(1:18);
    Fc_solution = solution(19:end);

    %% --- 8. Final Torque Calculation ---
    tau_cmd = H_joint * q_ddot_solution + C_joint - Jc_T_joint * Fc_solution;
    
    contact_state = contact_cmd;
end