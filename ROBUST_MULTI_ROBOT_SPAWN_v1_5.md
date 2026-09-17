# Robust multi-robot startup v1.5

v1.5 adds a deterministic **post-controller upright recovery** stage for Gazebo
multi-Go2 startup.

Observed failure addressed:

- the model may rotate to >30 deg pitch before its effort controller is active;
- once the controller becomes active, CHAMP does not reliably recover a robot
  that is already resting in a tipped configuration;
- the scientific run guard then correctly remains in `STARTING` and eventually
  rejects the run as `missing_upright`.

Behavior in v1.5:

1. Spawn robot as before.
2. Start the service-driven controller bootstrap immediately.
3. Activate joint-state and joint-trajectory controllers as before.
4. For 3.0 wall-clock seconds by default, repeatedly restore the Gazebo model
   to the requested spawn position/orientation using `/world/<name>/set_pose`.
5. Release the pose recovery and let the normal run guard require continuous
   upright stability before the scientific episode begins.

The recovery stage is startup-only; it occurs before `experiment_ready` and
therefore does not alter scientific episode time, MILP assignments, OGR policy,
or monitoring metrics.

New launch arguments:

- `controller_pose_recovery_s` (default `3.0`)
- `controller_pose_recovery_period_s` (default `0.25`)
- `controller_pose_recovery_z_offset_m` (default `0.0`)

The v1.4 immediate service-driven bootstrap and serialized controller lock are
retained.
