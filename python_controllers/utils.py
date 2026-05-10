import numpy as np

def skew_matrix(v):
    """
    Returns the skew-symmetric matrix for cross product: S*x = v x x
    """
    return np.array([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ]
    ])

def dynamic_pinv(J, A):
    """
    Implements the dynamically consistent pseudo-inverse with DAMPING for numerical stability.
    J_bar_dyn = A^-1 * J^T * pinv(J * A^-1 * J^T + damping*I)
    """
    damping = 1e-6
    # A_inv_J_T = A \ J'
    A_inv_J_T = np.linalg.solve(A, J.T)
    
    # lambda_inv = J * A_inv_J_T
    lambda_inv = J @ A_inv_J_T
    
    m = lambda_inv.shape[0]
    
    # lambda = pinv(lambda_inv + damping*I)
    lambda_mat = np.linalg.pinv(lambda_inv + damping * np.eye(m))
    
    # Final damped inverse
    J_bar_dyn = A_inv_J_T @ lambda_mat
    return J_bar_dyn

def build_Ak(yaw, dt):
    """
    Builds the discrete-time state-transition matrix A_k
    """
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]
    ])
    
    Ak = np.eye(12)
    
    # Theta(k+1) = Theta(k) + Rz*omega*dt
    Ak[0:3, 6:9] = Rz * dt
    
    # p(k+1) = p(k) + p_dot*dt
    Ak[3:6, 9:12] = np.eye(3) * dt
    
    return Ak

def build_Bk(mass, I_body_inv, yaw, p_com, p_feet_world, dt, contact):
    """
    Builds the discrete-time control-input matrix B_k
    p_feet_world is 3x4
    contact is 4x1
    """
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [          0,            0, 1]
    ])
    
    # Global inertia tensor inverse
    gI_inv = Rz @ I_body_inv @ Rz.T
    
    Bk = np.zeros((12, 12))
    
    for i in range(4):
        if contact[i] == 1:
            f_idx = slice(i*3, i*3+3)
            
            # r_i = p_foot_i - p_com
            r_i = p_feet_world[:, i] - p_com
            
            r_cross_matrix = skew_matrix(r_i)
            
            # Rotational dynamics
            Bk[6:9, f_idx] = gI_inv @ r_cross_matrix * dt
            
            # Linear dynamics
            Bk[9:12, f_idx] = (np.eye(3) / mass) * dt
            
    return Bk