# State Machine: How It Works (English)

## Overview

This document explains, in a clear and practical way, how the mission state machine governs the AUV terrain-following behavior. It covers the execution flow inside the main loop, the diagnostic flags carried in the `cmd` structure, the states and transitions (including recovery logic), the main tunable parameters, and how to test typical failure scenarios.

The design goals are robustness to sensor loss, predictable recovery, and clean separation between behavior (state machine), goals (setpoints), control (PID), estimation (EKF), and environment (terrain generator).

---

## Execution Flow in main_6DOF_3D.m

At each simulation step k, the following happens in order:

1) State selection
   next_state = state_machine(state(k-1), cmd, k)
   Uses the diagnostic flags computed in the previous iteration to select the current state.

2) Goal generation
   goal(k) = goal_def(state(k), rob_rot(:,k-1), x_est(:,k-1), k)
   Produces target surge, sway, altitude, roll, pitch, yaw for the chosen state.

3) Control (PID)
   pid(:,k) = input_control(goal(k), x_est(:,k-1), rob_rot(:,k-1), ...)
   Computes control references with anti-windup, including frame transforms.

4) Robot dynamics and sensors
   - u(:,k) = dynamic_model(...)
   - [clean_rot(:,k), wRr_real] = AHRS_measurement(...)
   - [dvl_speed_w, prob(:,k)]   = DVL_measurament(...)

5) Terrain update and SBES
   - [plane, t_idx] = terrain_generator(...)
   - [z_meas(:,k), h_true, n_mes(:,k), R(:,:,k), cmd, a_true, b_true] = SBES_measurament(...)
   SBES updates: cmd.contact(1:4) and cmd.sensor_fail.

6) EKF prediction and update
   x_pred(:,k), P_pred → z_pred(:,k), K, x_est(:,k), P

7) Diagnostic update for next step
   cmd = goal_controller(cmd, x_est(:,k), rob_rot(:,k), goal(k), N, state(k), k, d_dim)
   Computes diagnostic flags that the state machine will use at k+1.

Timing summary:
- At iteration k: state_machine uses cmd from (k-1)
- SBES sets raw contact info at k
- goal_controller computes diagnostic flags at k
- At k+1: state_machine uses the new diagnostic flags

---

## Command Structure (cmd): Diagnostic Flags

Initialization (in main):

```matlab
cmd.pitch_sensors_lost = false;     % Sensors 1-2 both lost (pitch pair)
cmd.roll_sensors_lost  = false;     % Sensors 3-4 both lost (roll pair)
cmd.diagonal_sensors_lost = false;  % Diagonal loss (1-4 or 2-3)
cmd.sensor_fail_persistent = false; % Failure persisted beyond grace period
cmd.recovery_timeout = false;       % Recovery maneuver timed out
cmd.recovery_progress = false;      % Failure condition improving during recovery
```

How they are computed (in goal_controller.m):
- pitch_sensors_lost  = ~cmd.contact(1) && ~cmd.contact(2)
- roll_sensors_lost   = ~cmd.contact(3) && ~cmd.contact(4)
- diagonal_sensors_lost = (~cmd.contact(1) && ~cmd.contact(4)) || (~cmd.contact(2) && ~cmd.contact(3))
- sensor_fail_persistent = true after a grace period (e.g., 500 steps)
- recovery_timeout = true after a timeout (e.g., 5000 steps) in a recovery state
- recovery_progress = true if sensor_fail decreases while recovering

Note: SBES_measurament sets `cmd.contact(1:4)` and increments `cmd.sensor_fail` when a sensor has no valid intersection.

---

## States and Meanings

Existing/improved states:
1. Idle → TargetAltitude
2. TargetAltitude → ContactSearch
3. ContactSearch → Following / MovePitch / MoveRoll / RecoveryAltitude / Reset
4. MovePitch → ContactSearch / RecoveryAltitude (on timeout)
5. MoveRoll → ContactSearch / RecoveryAltitude (on timeout)
6. Following → MovePitch / MoveRoll / RecoveryAltitude / Reset
7. Emergency → TargetAltitude
8. EndSimulation → EndSimulation

New state:
9. RecoveryAltitude → ContactSearch / Reset (on timeout)
   - Lowers the target altitude slightly (e.g., −1 m) to regain SBES contact
   - Uses very slow motion to avoid destabilization
   - Used for diagonal loss and as fallback after MovePitch/MoveRoll timeouts

---

## Key Transitions (Logic Summary)

ContactSearch → Recovery family:

```
if sensor_fail == 0             → Following
elseif sensor_fail == 4         → Reset
elseif sensor_fail >= 2
    if pitch_sensors_lost       → MovePitch
    elseif roll_sensors_lost    → MoveRoll
    elseif diagonal_sensors_lost→ RecoveryAltitude
    else                        → ContactSearch (stay)
end
```

Following → Recovery family (only if persistent):

```
if sensor_fail >= 2 && sensor_fail_persistent
    if pitch_sensors_lost       → MovePitch
    elseif roll_sensors_lost    → MoveRoll
    elseif diagonal_sensors_lost→ RecoveryAltitude
    else                        → Following (transient loss)
end
```

Recovery escalation and exit:

```
MovePitch / MoveRoll:
  if recovery_timeout          → RecoveryAltitude
  elseif both sensors recovered→ ContactSearch
  else                         → stay

RecoveryAltitude:
  if recovery_timeout          → Reset
  elseif sensor_fail == 0      → ContactSearch
  else                         → stay
```

Emergency and end-state transitions are safety/termination overrides applied from anywhere as needed.

---

## Tunable Parameters (Where to Look)

In goal_controller.m:

```matlab
recovery_timeout_steps = 5000;  % 5 s at Ts = 1 ms
grace_period_steps     = 500;   % 0.5 s transient tolerance
```

In goal_def.m (used during recovery states):

```matlab
% Recovery speeds
u_star_recovery = 0.02;     % Surge [m/s]
v_star_recovery = -0.05;    % Sway  [m/s]

% Angular adjustments
ang_to_cut_initial    = pi/12; % 15 deg for small errors
ang_to_cut_progressive= pi/8;  % 22.5 deg for large errors

% Altitude adjustment in RecoveryAltitude
altitude_offset = 1.0;      % meters (target altitude reduced by this amount)
```

---

## Failure Patterns Covered

1) Same-axis loss (pitch or roll):
   - Sensors 1-2 lost → MovePitch
     Gradual pitch adjustment, very slow motion, 5 s timeout → RecoveryAltitude.
   - Sensors 3-4 lost → MoveRoll
     Gradual roll adjustment, very slow motion, 5 s timeout → RecoveryAltitude.

2) Diagonal loss (1-4 or 2-3):
   - Directly → RecoveryAltitude
     Reduce altitude target, move very slowly, 5 s timeout → Reset if no improvement.

3) Total loss (all 4 sensors):
   - → Reset
     Return to target altitude and reset attitude, then retry ContactSearch.

4) Transient loss (< grace period):
   - Ignored by design (no transition) to avoid oscillations on brief glitches.

---

## Suggested Test Scenarios

Scenario 1: Pitch sensors loss (1-2)
1. Start in Following.
2. Force loss of sensors 1-2 for > 0.5 s.
3. Expect: Following → MovePitch → ContactSearch (if recovered) or RecoveryAltitude (if timeout).

Scenario 2: Roll sensors loss (3-4)
1. Start in Following.
2. Force loss of sensors 3-4 for > 0.5 s.
3. Expect: Following → MoveRoll → ContactSearch (if recovered) or RecoveryAltitude (if timeout).

Scenario 3: Diagonal loss (1-4 or 2-3)
1. Start in Following.
2. Force diagonal pair loss.
3. Expect: → RecoveryAltitude → ContactSearch (if recovered) or Reset (if timeout).

Scenario 4: Transient glitch (< 0.5 s)
1. Start in Following.
2. Drop one or more sensors briefly (< grace period).
3. Expect: remain in Following (no transition), then normal operation.

---

## Persistent Variables (goal_controller.m)

The controller keeps small cross-iteration memory to track recovery timing and progress:

```matlab
persistent recovery_start_step;        % When recovery started
persistent last_sensor_fail_count;     % Previous failure count
persistent sensor_fail_grace_counter;  % Grace period counter
```

These are automatically reset when leaving recovery-related states. They enable timeout behavior and “progress” checks without external bookkeeping.

---

## Compatibility Notes

Files updated for this logic:
1. main_6DOF_3D.m      — new cmd fields and flow already integrated
2. state_machine.m      — transitions for MovePitch/MoveRoll/RecoveryAltitude
3. goal_def.m           — setpoints for recovery behaviors
4. goal_controller.m    — diagnostic flags, grace period, timeout handling

No changes required in:
- SBES_measurament.m (already sets contact and failure counts)
- input_control.m, dynamic_model.m, EKF functions (use their existing contracts)

---

## Benefits at a Glance

- Robustness: Transient glitches are filtered by the grace period.
- Safety: Timeouts guarantee exit from stalled recovery.
- Adaptivity: Angle corrections sized by current error magnitude.
- Coverage: Handles same-axis, diagonal, and total sensor failures.
- Traceability: Console logs for every transition (step and reason).
- Hierarchy: Clear fallback chain → MovePitch/Roll → RecoveryAltitude → Reset.

---

## Debug Tips

Enable detailed logging (optional additions in goal_controller.m):

```matlab
fprintf('Step %d: sensor_fail=%d, persistent=%d, pitch_lost=%d, roll_lost=%d\n', ...
        step, command.sensor_fail, command.sensor_fail_persistent, ...
        command.pitch_sensors_lost, command.roll_sensors_lost);

fprintf('Recovery tracking: start=%d, grace_counter=%d, timeout=%d\n', ...
        recovery_start_step, sensor_fail_grace_counter, command.recovery_timeout);
```

---

## Conclusion

This state machine provides a resilient behavior layer for underwater terrain following:

- Automatic, multi-level recovery from different sensor loss patterns
- Predictable fallback with timeouts and safe defaults
- Minimal cross-coupling with control/estimation layers
- Simple tuning (two time constants + conservative recovery goals)

It is ready for long runs, batch experiments, and reproducible analysis with the provided data management tools.

—

Last updated: 2025-10-23