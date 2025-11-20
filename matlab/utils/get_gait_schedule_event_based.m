function [contact, swing_phase, stance_phase, state_changed, p_touchdown] = get_gait_schedule_event_based(...
    timer, T_cycle, stance_percent, phase_offset, foot_force, force_threshold, ...
    prev_contact, prev_touchdown, current_foot_pos)
    % Implements Event-Based Finite State Machine with proprioceptive touch-down detection
    % (Bledt et al. 2018, Section III-A + Hyun et al. 2014)
    %
    % Combines periodic phase-based scheduling with force-based event detection for:
    %   - Early touch-down detection during swing (uneven terrain)
    %   - Robust state transitions based on actual ground contact
    %   - Improved terrain adaptation
    %
    % Inputs:
    %   timer            - Current gait timer for this leg [s]
    %   T_cycle          - Full gait cycle period [s]
    %   stance_percent   - Stance duty factor ∈ [0, 1] (e.g., 0.5 for trot)
    %   phase_offset     - Phase offset for this leg ∈ [0, 1]
    %   foot_force       - Current foot contact force magnitude [N]
    %   force_threshold  - Touch-down detection threshold [N] (typically 20-30 N)
    %   prev_contact     - Previous contact state (0=swing, 1=stance)
    %   prev_touchdown   - [3x1] Previous touch-down position
    %   current_foot_pos - [3x1] Current foot position
    %
    % Outputs:
    %   contact      - Contact state (0=swing, 1=stance)
    %   swing_phase  - Normalized swing phase ∈ [0, 1] (0 during stance)
    %   stance_phase - Normalized stance phase ∈ [0, 1] (0 during swing)
    %   state_changed - Boolean, true if contact state changed this timestep
    %   p_touchdown  - [3x1] Touch-down position (updated on swing->stance transition)

    %% --- Step 1: Periodic Phase-Based Schedule ---
    % This is the baseline schedule based on gait timing
    phase = mod((timer / T_cycle) + phase_offset, 1.0);

    % Default schedule based on phase
    if phase < stance_percent
        scheduled_contact = 1;  % Stance phase
        stance_phase = phase / stance_percent;
        swing_phase = 0.0;
    else
        scheduled_contact = 0;  % Swing phase
        stance_phase = 0.0;
        swing_phase = (phase - stance_percent) / (1.0 - stance_percent);
    end

    %% --- Step 2: Event-Based Override (Touch-Down Detection) ---
    % If we're scheduled to be in swing BUT significant force is detected,
    % transition immediately to stance (early touch-down on uneven terrain)

    contact = scheduled_contact;
    p_touchdown = prev_touchdown;

    if scheduled_contact == 0 && foot_force > force_threshold
        % Early touch-down detected during swing!
        % Override: switch to stance immediately
        contact = 1;
        swing_phase = 0.0;
        stance_phase = 0.0;  % Reset stance phase at touch-down event

        % Record new touch-down position
        p_touchdown = current_foot_pos;
    end

    %% --- Step 3: Detect State Transitions ---
    % Track swing->stance and stance->swing transitions
    state_changed = (contact ~= prev_contact);

    % Update touch-down position on swing->stance transition
    if state_changed && contact == 1 && prev_contact == 0
        % Transitioning from swing to stance: record touch-down position
        p_touchdown = current_foot_pos;
    end

    %% --- Additional Logic: Late Lift-Off Detection (Optional) ---
    % Could also add logic for delaying lift-off if large forces persist
    % beyond scheduled lift-off time (e.g., robot stuck on obstacle).
    % For now, we trust the periodic scheduler for lift-off timing.
end
