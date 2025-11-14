function J_bar_dyn = dynamic_pinv(J, A)
    % Implements the dynamically consistent pseudo-inverse
    % with DAMPING for numerical stability.
    
    % Damping factor (small number to prevent singularity)
    damping = 1e-6; 
    
    % --- Calculate Task-Space Inertia (lambda) ---
    % Original: lambda_inv = J * (A \ J');
    % We will add the damping term here.
    
    A_inv_J_T = A \ J'; % Equivalent to inv(A) * J'
    lambda_inv = J * A_inv_J_T;
    
    % --- Add Damping ---
    % Get the size of the square matrix lambda_inv
    [m, ~] = size(lambda_inv); 
    
    % Original: lambda = pinv(lambda_inv);
    % New: lambda = pinv(lambda_inv + damping*I)
    lambda = pinv(lambda_inv + damping * eye(m)); 
    
    % --- Calculate Final Damped Inverse ---
    J_bar_dyn = A_inv_J_T * lambda;
end