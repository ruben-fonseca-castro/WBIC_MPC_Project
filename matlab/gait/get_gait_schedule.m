function [contact, swing_phase, stance_phase, state_changed, p_touchdown] = get_gait_schedule(...
    timer, T_cycle, stance_percent, phase_offset, foot_force, force_threshold, ...
    prev_contact, prev_touchdown, current_foot_pos)
    % Gait scheduler with optional event-based touch-down detection
    % (Bledt et al. 2018, Section III-A + Hyun et al. 2014)
    %
    % SIMPLE CALL (periodic only):
    %   [contact, swing_phase] = get_gait_schedule(timer, T_cycle, stance_percent, phase_offset)
    %
    % FULL CALL (with event-based detection):
    %   [contact, swing_phase, stance_phase, state_changed, p_touchdown] = get_gait_schedule(
    %       timer, T_cycle, stance_percent, phase_offset, foot_force, force_threshold,
    %       prev_contact, prev_touchdown, current_foot_pos)
    %
    % Inputs:
    %   timer            - Current gait timer [s]
    %   T_cycle          - Full gait cycle period [s]
    %   stance_percent   - Stance duty factor ∈ [0, 1]
    %   phase_offset     - Phase offset for this leg ∈ [0, 1]
    %   foot_force       - (Optional) Current foot force magnitude [N]
    %   force_threshold  - (Optional) Touch-down threshold [N]
    %   prev_contact     - (Optional) Previous contact state
    %   prev_touchdown   - (Optional) [3x1] Previous touch-down position
    %   current_foot_pos - (Optional) [3x1] Current foot position
    %
    % Outputs:
    %   contact      - Contact state (0=swing, 1=stance)
    %   swing_phase  - Normalized swing phase ∈ [0, 1]
    %   stance_phase - Normalized stance phase ∈ [0, 1]
    %   state_changed - Boolean, true if contact state changed
    %   p_touchdown  - [3x1] Touch-down position

    %% --- Step 1: Periodic Phase-Based Schedule ---
    phase = mod((timer / T_cycle) + phase_offset, 1.0);

    if phase < stance_percent
        scheduled_contact = 1;  % Stance phase
        stance_phase = phase / stance_percent;
        swing_phase = 0.0;
    else
        scheduled_contact = 0;  % Swing phase
        stance_phase = 0.0;
        swing_phase = (phase - stance_percent) / (1.0 - stance_percent);
    end

    % Default outputs for simple call
    contact = scheduled_contact;
    state_changed = false;
    p_touchdown = zeros(3, 1);

    %% --- Step 2: Event-Based Override (if arguments provided) ---
    if nargin >= 9
        p_touchdown = prev_touchdown;

        % Early touch-down detection during swing
        if scheduled_contact == 0 && foot_force > force_threshold
            contact = 1;
            swing_phase = 0.0;
            stance_phase = 0.0;
            p_touchdown = current_foot_pos;
        end

        % Detect state transitions
        state_changed = (contact ~= prev_contact);

        % Update touch-down position on swing->stance transition
        if state_changed && contact == 1 && prev_contact == 0
            p_touchdown = current_foot_pos;
        end
    end
end
