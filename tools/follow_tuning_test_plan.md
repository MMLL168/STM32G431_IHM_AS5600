# Follow Tuning Test Plan

## Recommended Order

1. Direction Probe
   - Purpose: verify the inner sensored direction and electrical trim if the motor behavior becomes obviously wrong again.
   - Tool setting: test mode = direction_probe.
   - Preset: `00_direction_probe`
   - Key observations: rotation direction, smoothness, and whether the motor can produce stable low-speed torque.

2. Fixed Target Hold
   - Purpose: test whether the outer loop behaves like a spring around one target without knob noise.
   - Tool setting: test mode = fixed_target.
   - Preset: `01_fixed_target_hold`
   - Suggested offset: 0 first, then `02_fixed_target_offset_150`.
   - Key observations: when the shaft is disturbed by hand, it should return and settle instead of hunting.

3. Step Sequence Small
   - Purpose: tune near-target convergence and damping.
   - Tool setting: test mode = step_sequence.
   - Preset: `03_step_small_120`
   - Suggested amplitude: 100 to 150 deg10.
   - Suggested dwell: 1200 to 1800 ms.
   - Key observations: overshoot count and whether the swing amplitude shrinks each cycle.

4. Step Sequence Medium
   - Purpose: tune large-angle energy and braking behavior.
   - Tool setting: test mode = step_sequence.
   - Preset: `04_step_medium_360`
   - Suggested amplitude: 300 to 600 deg10.
   - Key observations: first overshoot size, settling time, and whether the motor chooses the short path.

5. Normal Follow
   - Purpose: only after fixed_target and step_sequence are convergent.
   - Tool setting: test mode = normal_follow.
   - Preset: `05_normal_follow`
   - Key observations: small knob motion should feel continuous, and larger knob motion should settle like a damped pendulum.

## Parameter Focus

1. Spring strength
   - position_current_ma_num / den
   - Too low: weak return, may stall near target.
   - Too high: aggressive overshoot.

2. Damping
   - damping_current_ma_per_rpm
   - Too low: repeated oscillation.
   - Too high: sluggish and sticky.

3. Breakaway
   - breakaway_current_ma
   - breakaway_speed_window_rpm
   - Use only enough to overcome static friction.

4. Input trajectory
   - target_angle_slew_min/max/divisor/accel/decel
   - These shape how aggressively the command angle moves.

5. Near-target envelope
   - settle_zone_deg10
   - settle_max_rpm
   - settle_max_current_ma
   - These should make the last part of motion calmer, not unstable.