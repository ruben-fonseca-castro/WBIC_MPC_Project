# WBIC Force Tracking Validation Test Guide

## Overview

This test validates that WBIC (Whole-Body Impulse Control) correctly tracks force commands from MPC by:
1. Generating sinusoidal force redistributions (while maintaining total force = mg)
2. Commanding WBIC to execute these forces
3. Measuring body response and validating proper physics

## Prerequisites

- Robot must be in a **safe standing position**
- All sensors operational (IMU, encoders, foot contacts)
- WBIC controller running at 500 Hz
- Robot should start approximately level (pitch/roll < 5°)

## Running the Test

### 1. Start the Robot Simulator/Hardware
```bash
# In terminal 1: Start your robot bridge
cd arc_bridge
python bridges/unitree_a1_bridge.py
```

### 2. Run the WBIC Test
```matlab
% In MATLAB:
cd /home/davidho/arc-bridge/matlab
test_wbic_force_tracking
```

### 3. Monitor Progress

The test will:
- Run for **10 seconds** (configurable via `TEST_DURATION`)
- Print progress every 0.5 seconds
- Generate plots automatically when complete
- Save results to `wbic_test_results.mat`

## Understanding the Results

### Pass Criteria

The test evaluates **5 key metrics**:

| Test | Pass Threshold | What It Validates |
|------|----------------|-------------------|
| **Force Conservation** | Error < 0.1 N | Total vertical force = mg |
| **Force Tracking** | Error < 5 N | WBIC QP tracks MPC commands |
| **Pitch-Force Coupling** | Correlation > 0.3 | Body responds to force redistribution |
| **Torque Limits** | Max τ < 33.5 Nm | Joint torques reasonable |
| **Stability** | Drift < 0.1 m, 5° | No runaway behavior |

### Expected Behavior

✅ **PASS Indicators:**
- Body pitch oscillates in sync with front-rear force changes
- Body roll oscillates in sync with left-right force changes
- Yaw remains stable (no drift)
- Forces commanded ≈ forces executed (dashed lines track solid lines in Plot 1)
- Joint torques stay well below 33.5 Nm

⚠️ **WARNING Indicators:**
- Pitch-force correlation < 0.3 → Body not responding properly
- Large force tracking error (> 5N) → WBIC QP having issues
- Torques near limits → Gains may be too high

❌ **FAIL Indicators:**
- Force conservation violated → Physics constraint broken
- Position drift > 0.1m → Instability
- Torques exceed 33.5 Nm → Hardware limits exceeded

## Interpreting the Plots

### Plot 1: Force Commands vs Finals
- **Dashed lines** = MPC commands (input to WBIC)
- **Solid lines** = WBIC final forces (output of QP)
- Should track closely (< 5N difference)
- Small deviations are OK (WBIC adjusts for body control)

### Plot 2-3: Force Distribution
- Shows how forces are redistributed between legs
- Front-rear creates pitch moments
- Left-right creates roll moments
- Total force should oscillate around mg

### Plot 4-5: Orientation Response ⭐ **KEY PLOTS**
- **Blue line** = Body orientation (roll/pitch)
- **Red dashed** = Force difference driving the moment
- Should show **clear correlation** (when force diff increases, pitch/roll changes)
- **Phase lag is normal** (body takes time to respond)

### Plot 6: Yaw Stability
- Should remain nearly flat
- Any drift indicates control issues or state estimation error

### Plot 7: Joint Torques
- All should stay well below 33.5 Nm red line
- If near limits → reduce gains or force amplitudes

### Plot 8: Expected Moments
- Shows the moment created by force distribution
- My (pitch moment) should oscillate at FREQ_PITCH
- Mx (roll moment) should oscillate at FREQ_ROLL

### Plot 9: Position Drift
- Should stay within ±50mm of starting position
- Larger drift suggests instability or poor tracking

## Troubleshooting

### Issue: "Tests Passed: 0 / 5"

**Possible Causes:**
1. Robot not standing properly initially
2. WBIC gains too low (no response) or too high (oscillations)
3. State estimator not converged
4. Friction cone constraints too restrictive

**Debug Steps:**
```matlab
% Check initial state
% At test start, verify:
%   - All 4 feet in contact
%   - Pitch/roll < 5°
%   - Height ≈ 0.35m

% Check WBIC QP solve rate
% Look for "⚠ QP failed" messages in WBIC debug output
```

### Issue: "⚠ PITCH ERROR but F-R force diff only 0.3 N"

**This is the exact issue you're experiencing!**

This means:
- MPC is commanding small force differences
- Body has large pitch error but forces aren't correcting it

**Likely Causes:**
1. ✅ **WBIC QP friction cone on all legs** (we fixed this!)
2. MPC orientation gains too low
3. State estimation drift causing MPC linearization error

**Validation:**
If this test shows **strong pitch-force correlation (>0.3)**, then WBIC works correctly and the issue is in MPC!

### Issue: Force tracking error > 10N

**Possible Causes:**
1. WBIC QP weights (Q1, Q2) not tuned properly
2. Friction cone too restrictive
3. Task hierarchy conflicts

**Fix:**
```matlab
% In run_wbic_controller.m, try:
Q1 = 0.5 * eye(12);  % Reduce force tracking weight
Q2 = 0.2 * eye(6);   % Increase floating base freedom
```

### Issue: Torques exceed 33.5 Nm

**Fix:**
```matlab
% In test_wbic_force_tracking.m, reduce test amplitudes:
AMPLITUDE_PITCH = 5;  % Was 10
AMPLITUDE_ROLL = 4;   % Was 8
```

## Customizing the Test

### Change Test Parameters

```matlab
% In test_wbic_force_tracking.m, edit:

TEST_DURATION = 20.0;      % Longer test (default: 10s)
FREQ_PITCH = 0.3;          % Slower oscillation (default: 0.5 Hz)
AMPLITUDE_PITCH = 15;      % Larger moments (default: 10 N)
```

### Test Different Scenarios

**Scenario 1: Pure Pitch Test**
```matlab
AMPLITUDE_ROLL = 0;   % Disable roll test
FREQ_PITCH = 0.5;     % Focus on pitch only
```

**Scenario 2: Step Response**
```matlab
% Replace sinusoid with step:
% Line ~125 in test script:
if t < 2.0
    delta_pitch = 0;
elseif t < 7.0
    delta_pitch = 10;  % Step up
else
    delta_pitch = 0;   % Step down
end
```

## Expected Terminal Output

```
========================================
          ANALYSIS & RESULTS
========================================

Force Conservation:
  Total Fz: 122.05 ± 0.02 N (expected: 122.10 N)
  Max error: 0.08 N
  ✅ PASS: Force is conserved

Body Orientation Response:
  Roll range:  3.21° (expected: some oscillation)
  Pitch range: 4.15° (expected: some oscillation)
  Yaw range:   0.42° (expected: minimal)

Force Tracking (MPC cmd → WBIC final):
  RMS error per component:
    Fx: 0.124 N, Fy: 0.098 N, Fz: 0.876 N (per leg avg)
  Max Fz error: 2.31 N
  ✅ PASS: WBIC tracks MPC forces well (<5N error)

Force-Orientation Coupling:
  Pitch-Force correlation: 0.687
  ✅ PASS: Body pitch responds to force changes

Torque Analysis:
  Max joint torque: 18.23 Nm (limit: 33.5 Nm)
  Mean joint torque: 8.45 Nm
  ✅ PASS: Torques within limits

Stability Check:
  Position drift: 0.032 m
  Orientation drift: 1.84 deg
  ✅ PASS: System stable (no runaway)

========================================
          SUMMARY
========================================

Tests Passed: 5 / 5
✅ ALL TESTS PASSED!
WBIC is correctly tracking force commands.
```

## Next Steps After This Test

### If ALL TESTS PASS ✅
→ **WBIC is working correctly!**
→ The pitch control issue is likely in **MPC**, not WBIC
→ Investigate MPC orientation drift (see main conversation)

### If Force Tracking FAILS ❌
→ Debug WBIC QP formulation
→ Check friction cone constraints
→ Verify task hierarchy

### If Pitch-Force Coupling is WEAK ⚠️
→ Check WBIC task gains (kp_base, kd_base)
→ Verify state estimator quality
→ Check if foot positions are accurate

## Files Generated

After test completion:
- `wbic_test_results.mat` - Full data log (can reload for re-plotting)
- Figure window with 9 plots

To reload results later:
```matlab
load('wbic_test_results.mat');
% Analyze log struct
```

---

**Questions or issues?** Check WBIC debug output during test for QP solver failures.
