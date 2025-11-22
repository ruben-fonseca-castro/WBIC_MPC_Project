function [Bk] = build_Bk(mass, I_body_inv, yaw, p_com, p_feet_world, dt, contact)
    % Builds the discrete-time control-input matrix B_k
    % Based on Eq 9
    %
    % mass:         Robot mass (scalar)
    % I_body_inv:   3x3 inverse inertia tensor in body frame
    % yaw:          Current body yaw (scalar)
    % p_com:        Current 3x1 body position (world frame)
    % p_feet_world: Current 3x4 matrix of foot positions (world frame)
    % dt:           Time step (scalar)
    % contact:      4x1 contact schedule for this step

    % Rotation from body to world (Eq 5)
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw),  cos(yaw), 0;
                 0,         0, 1];
    
    % Global inertia tensor inverse (Eq 5)
    % gI_inv = Rz(psi) * bI_inv * Rz(psi)'
    gI_inv = Rz * I_body_inv * Rz';
    
    Bk = zeros(12, 12); % 12x12
    
    for i = 1:4
        if contact(i) == 1
            % This foot is in STANCE
            f_idx = (i-1)*3 + (1:3); % Columns for f_i
            
            % Get moment arm, r_i = p_foot_i - p_com
            r_i = p_feet_world(:, i) - p_com;
            
            % Create the cross-product matrix for r_i
            r_cross_matrix = [  0,   -r_i(3),  r_i(2);
                              r_i(3),   0,   -r_i(1);
                             -r_i(2), r_i(1),   0   ];
            
            % Rotational dynamics (Eq 9)
            % omega_dot = gI_inv * (r_i x f_i)
            % omega(k+1) = omega(k) + (gI_inv * r_cross * f_i) * dt
            Bk(7:9, f_idx) = gI_inv * r_cross_matrix * dt;
            
            % Linear dynamics (Eq 9)
            % p_dot_dot = (1/m) * f_i
            % p_dot(k+1) = p_dot(k) + (1/m)*f_i*dt
            Bk(10:12, f_idx) = (eye(3) / mass) * dt;
        end
        % If contact(i) == 0, columns of Bk remain zero,
        % "eliminating" the variable.
    end
end