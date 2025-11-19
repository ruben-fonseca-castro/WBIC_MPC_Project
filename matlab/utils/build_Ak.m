function [Ak] = build_Ak(yaw, dt)
    % Builds the discrete-time state-transition matrix A_k
    % Based on Eq 9
    
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw),  cos(yaw), 0;
                 0,         0, 1];
    
    % Start with Identity
    Ak = eye(12);
    
    % Theta(k+1) = Theta(k) + Rz*omega*dt
    % (Simplification from Eq 4: Theta_dot = Rz(psi)*omega)
    Ak(1:3, 7:9) = Rz * dt;
    
    % p(k+1) = p(k) + p_dot*dt
    Ak(4:6, 10:12) = eye(3) * dt;
end