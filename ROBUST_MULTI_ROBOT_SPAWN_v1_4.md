# Robust multi-robot startup v1.4

The controller bootstrap waiter now starts immediately with each robot group.
It still blocks until the per-robot controller-manager service exists and retains
the serialized `flock` bootstrap from v1.3. This removes the fixed 14 s window
in which a spawned dynamic Go2 could settle or overturn before joint control
was activated.
