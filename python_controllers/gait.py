import numpy as np
import math

def get_gait_schedule(timer, T_cycle, stance_percent, phase_offset):
    """
    Simple gait scheduler (periodic only).
    Returns (contact, swing_phase)
    """
    if phase_offset > 1.0:
        return 1, 0.0
    
    phase = ((timer / T_cycle) + phase_offset) % 1.0
    
    if phase < stance_percent:
        return 1, 0.0
    else:
        swing_phase = (phase - stance_percent) / (1.0 - stance_percent)
        return 0, swing_phase

def get_stance_trajectory(p_touchdown, stance_phase, T_stance, delta_amplitude):
    """
    Implements Equilibrium-Point Hypothesis for stance trajectory.
    Returns (p_stance, v_stance)
    """
    z_offset = delta_amplitude * np.sin(np.pi * stance_phase)
    vz_offset = delta_amplitude * (np.pi / T_stance) * np.cos(np.pi * stance_phase)
    
    p_stance = np.copy(p_touchdown)
    p_stance[2] += z_offset
    
    v_stance = np.zeros(3)
    v_stance[2] = vz_offset
    
    return p_stance, v_stance

def get_footstep_target(state_velocity, v_des, omega_des, p_shoulder, T_stance, k_raibert, current_height, gravity, swing_phase, T_swing):
    """
    Implements Foot Step Planner
    """
    v_curr = state_velocity[0:3]
    
    # Kinematic feedforward: expected shoulder position at touchdown
    time_left_in_swing = T_swing * (1.0 - swing_phase)
    p_shoulder_td = p_shoulder + v_curr * time_left_in_swing
    
    # 1. Symmetry Term
    p_symmetry = (T_stance / 2.0) * v_curr
    
    # 2. Feedback Term
    gain_vector = np.array([k_raibert, k_raibert * 2.0, 0])
    p_feedback = gain_vector * (v_curr - v_des)
    
    # 3. Centrifugal Term
    coeff = 0.5 * np.sqrt(current_height / gravity)
    p_centrifugal = coeff * np.cross(v_curr, omega_des)
    
    p_target = p_shoulder_td + p_symmetry + p_feedback + p_centrifugal
    p_target[2] = 0.0
    
    return p_target

def get_swing_trajectory_bezier(p_start, p_target, swing_phase, swing_height, T_swing):
    """
    Implements 12-point Bezier curve swing trajectory.
    """
    n = 11
    P = np.zeros((3, 12))
    
    for i in range(12):
        alpha = i / 11.0
        P[0:2, i] = (1 - alpha) * p_start[0:2] + alpha * p_target[0:2]
        
        # Base linear interpolation for Z
        P[2, i] = (1 - alpha) * p_start[2] + alpha * p_target[2]
        
        # Add relative swing height profile
        if 5 <= i <= 7:
            P[2, i] += swing_height
        elif 1 < i < 5:
            beta = (i - 1) / 4.0
            P[2, i] += swing_height * (beta**2)
        elif 7 < i < 10:
            beta = (10 - i) / 3.0
            P[2, i] += swing_height * (beta**2)
            
    u = swing_phase
    p_foot = np.zeros(3)
    v_foot_unnormalized = np.zeros(3)
    
    # Position
    for i in range(n + 1):
        B_i = math.comb(n, i) * (u**i) * ((1-u)**(n-i))
        p_foot += B_i * P[:, i]
        
    # Velocity
    for i in range(n):
        B_i_prime = math.comb(n-1, i) * (u**i) * ((1-u)**(n-1-i))
        v_foot_unnormalized += n * B_i_prime * (P[:, i+1] - P[:, i])
        
    v_foot = v_foot_unnormalized / T_swing
    return p_foot, v_foot