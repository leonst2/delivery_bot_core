# delivery_bot_bringup

Starts the whole stack with one command, the same way on the simulation and the robot, and monitors every node.

```bash
ros2 launch delivery_bot_bringup stack.launch.py sim:=true|false map:=<path>
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `sim` | `false` | Adds the `params/sim.yaml` overlay |
| `map` | empty | Path of the static map, passed to the nodes as `map_path` |
| `localization` | `mcl` | `mcl` or `ground_truth` (simulation runs only) |
| `perception`, `mapping`, `planning`, `executive` | `true` | Set to `false` to skip that stage for partial runs |

## Nodes

`delivery_bot_bringup/stack.py` lists every node with its stage and respawn policy. Nodes whose card hasn't landed
are `stub_node`, which only publishes heartbeats. A card replaces `package` and `executable` in its row.

Stages start in dependency order: description → odometry → localization → perception → mapping → planning →
executive. A stage starts once every node of the previous stage has sent a heartbeat. If that takes more than 30 s,
the launch shuts down.

## Parameters

`params/common.yaml` is shared by sim and robot. `params/sim.yaml` may only hold simulation differences, which
`test/test_params.py` enforces.

## Health

- Every stack node publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat` at >= 2 Hz. Python nodes use
  `HeartbeatPublisher` from `heartbeat.py`.
- `health_monitor` publishes `/health` (latched `diagnostic_msgs/DiagnosticArray`, one status per node, sorted by
  name). A node without a heartbeat for 1 s gets level `STALE` and the message `stalled`.
- `ros2 run delivery_bot_bringup health_check --timeout 30` exits 0 once every node is OK.

Heartbeats and the monitor run on the **steady clock**, not on ROS time. With `use_sim_time`, ROS timers stop without
`/clock`, so healthy nodes would look stalled until Unity enters Play mode, and again whenever it pauses.

Stop the launch with SIGINT. On SIGTERM, `ros2 launch` exits without stopping its nodes.
