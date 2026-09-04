# Robust multi-robot spawn v1.3

This revision keeps the staggered multi-Go2 startup from v1.2 and makes the
controller bootstrap idempotent.

`gz_ros2_control` may already load or activate controllers before the delayed
bootstrap runs. The launcher now queries controller state first:

- `active`: leave the controller unchanged;
- loaded but not active: transition it to `active` with `ros2 control set_controller_state`;
- missing: load it with `ros2 control load_controller --set-state active`.

This removes the false-failure path where the controller spawner reported
"Controller already loaded" followed by "Failed to configure controller".
No Go2 model, gait, control gains, topic, namespace, or spawn geometry is changed.
