# Robust multi-robot spawn patch v1.2

This revision is intended for OGR-PM Gazebo validation with 2--6 Unitree Go2 robots.

Changes relative to v1.1:

1. Gazebo receives a startup grace period before robot 0 is created (`robot_spawn_start_delay_s`, default 6 s).
2. Complete robot groups are started with a configurable stagger (`robot_start_stagger_s`, default 12 s).
3. Controller bootstrap is service-driven rather than relying on several independent fixed timers.
4. Controller loading is serialized across robots with a file lock to avoid controller-manager spawner lock contention.
5. The bootstrap waits explicitly for each `/<robot>/controller_manager/list_controllers` service before loading controllers.

No robot geometry, gait parameters, controller gains, topic names, or spawn poses are changed. The changes affect startup synchronization only.
