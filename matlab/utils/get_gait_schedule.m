function [contact, swing_phase] = get_gait_schedule(timer, T_cycle, stance_percent, phase_offset)
    % Implements the "Periodic Phase-Based State Scheduler"
    % [cite: image_9a0341.png]
    
    % Calculate this leg's specific phase in the gait cycle (0.0 to 1.0)
    phase = (timer / T_cycle) + phase_offset;
    if phase > 1.0
        phase = phase - 1.0;
    end
    
    if phase < stance_percent
        % STANCE PHASE
        contact = 1;
        swing_phase = 0.0; % Not in swing
    else
        % SWING PHASE
        contact = 0;
        % Calculate normalized swing_phase (0.0 to 1.0)
        swing_phase = (phase - stance_percent) / (1.0 - stance_percent);
    end
end