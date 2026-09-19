# Robust multi-robot startup v1.5

This revision fixes a controller lifecycle race observed after v1.4.

The controller-manager service can become visible before Gazebo has exported the
joint hardware interfaces, and a loaded controller may legitimately remain in
`unconfigured`.  Requesting `active` directly from `unconfigured` does not provide
a reliable lifecycle transition.

v1.5 therefore:

1. waits for `/controller_manager/list_controllers`;
2. serializes bootstrap across robots with the existing `flock` lock;
3. waits for the Go2 ros2_control position/effort hardware interfaces;
4. loads a missing controller;
5. explicitly transitions `unconfigured -> inactive` (configure);
6. explicitly transitions `inactive -> active` (activate);
7. retries the lifecycle state machine for up to 60 s; and
8. verifies that both controllers remain active continuously for 2 s.

This is an infrastructure/startup change only; it does not alter gait parameters,
controller gains, robot dynamics, or mission-level policy behavior.
