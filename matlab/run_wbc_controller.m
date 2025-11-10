function [tau_cmd, contact_state] = run_wbc_controller(state, params)
    
    %% --- Get Current & Desired State ---
    
    % Get current state
    q_pos_curr = state.qj_pos;
    q_vel_curr = state.qj_vel;

    q_des_final = params.q_des_base;

    %% --- Handle First-Loop Initialization ---
    if ~params.is_initialized
        % This is the first time we've run.
        % Set the ramp to the robot's CURRENT (lying down) position.
        params.q_des_ramping = q_pos_curr;
        params.is_initialized = true;
    end

    %% --- Calculate the Setpoint Ramp ---
    
    % Calculate the error between our final goal and our current ramp step
    ramp_error = q_des_final - params.q_des_ramping;
    
    % Calculate the next step, but limit its size
    step = max(min(ramp_error, params.max_step_size), -params.max_step_size);
    
    % Update the ramp setpoint
    params.q_des_ramping = params.q_des_ramping + step;

    %% --- Get Dynamics ---
    H_matrix = reshape(state.inertia_mat, [18, 18]);
    C_vector = state.bias_force;
    Jc = reshape(state.J_gc, [12, 18]);
    Jc_dot_q_dot = state.dJdq_gc;
    
    % Get the parts of H and C for the floating base (rows 1-6)
    H_base = H_matrix(1:6, :);
    C_base = C_vector(1:6);
    
    % Get the parts of H and C for the joints (rows 7-18)
    H_joint = H_matrix(7:18, :);
    C_joint = C_vector(7:18);
    
    % Get the transpose of Jc for the base and joints
    Jc_T_base = Jc(:, 1:6)';   % (12x6)
    Jc_T_joint = Jc(:, 7:18)'; % (12x12)

    %% --- 4. Define the Task: Desired Acceleration ---
    pos_error = params.q_des_ramping - q_pos_curr;
    vel_error = params.q_vel_des - q_vel_curr;
    
    q_j_ddot_des = params.KP_vec .* pos_error + params.KD_vec .* vel_error;
    q_ddot_des = [zeros(6,1); q_j_ddot_des]; % Full 18x1 desired acceleration

    %% --- 5. Setup the WBC Quadratic Program ---
    
    % --- Decision Variables ---
    % We solve for x = [q_ddot (18x1); Fc (12x1)]
    % This is a 30x1 vector
    n_q_ddot = 18;
    n_Fc = 12;
    n_vars = n_q_ddot + n_Fc;

    % --- Objective Function ---
    % min ||q_ddot - q_ddot_des||^2 + 0.001 * ||Fc||^2
    % (Try to match desired accel + Don't use too much force)
    
    H_qp = 2 * eye(n_vars);
    H_qp(1:n_q_ddot, 1:n_q_ddot) = 2 * eye(n_q_ddot); % Weight on accel error
    H_qp(n_q_ddot+1:end, n_q_ddot+1:end) = 2 * 0.001 * eye(n_Fc); % Small weight on force
    
    f_qp = zeros(n_vars, 1);
    f_qp(1:n_q_ddot) = -2 * q_ddot_des;

    % --- Equality Constraints: A_eq * x = b_eq ---
    % We have two *physical laws* that MUST be obeyed:
    
    % Constraint 1: Stance Foot Kinematics (12x30 matrix)
    % "The feet on the ground must not accelerate"
    % Jc*q_ddot + Jc_dot*q_dot = 0  =>  Jc*q_ddot = -Jc_dot*q_dot
    A_eq_stance = [ Jc, zeros(12, n_Fc) ];
    b_eq_stance = -Jc_dot_q_dot;

    % Constraint 2: Floating Base Dynamics (6x30 matrix)
    % "The floating base moves *only* due to ground forces and physics"
    % H_base*q_ddot + C_base = (Jc^T * Fc)_base
    % H_base*q_ddot - Jc_T_base*Fc = -C_base
    A_eq_dyn = [ H_base, -Jc_T_base ];
    b_eq_dyn = -C_base;

    % Combine constraints
    A_eq = [ A_eq_stance; A_eq_dyn ]; % (18x30) matrix
    b_eq = [ b_eq_stance; b_eq_dyn ]; % (18x1) vector

    % --- 6. Solve Quadratic Program ---
    options = optimoptions('quadprog', 'Display', 'off');
    
    % For this minimal fix, we'll ignore friction (A_ineq, b_ineq)
    [solution, fval, exitflag] = quadprog(H_qp, f_qp, [], [], A_eq, b_eq, [], [], [], options);
    
    if exitflag ~= 1
        disp('QP FAILED TO SOLVE! Commanding zero torques.');
        tau_cmd = zeros(12,1);
        contact_state = [1; 1; 1; 1];
        return;
    end
    
    % Extract the optimal solutions
    q_ddot_solution = solution(1:n_q_ddot);     % (18x1)
    Fc_solution = solution(n_q_ddot + 1 : end); % (12x1)

    %% --- Final Outputs ---
    tau_cmd = H_joint * q_ddot_solution + C_joint - Jc_T_joint * Fc_solution;
    
    % For a PD controller, we just assume all feet are in contact
    contact_state = [1; 1; 1; 1];
end