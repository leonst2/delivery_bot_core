# delivery_bot_map_server

Map server and map tools: own map format, latched /map and the LoadMap service.

| | |
| --- | --- |
| Card | [delivery_bot_core#12](https://github.com/leonst2/delivery_bot_core/issues/12) |
| Language | Python (rclpy) |
| Executable | `map_server` |

**Status: stub.** `map_server` only publishes `delivery_bot_interfaces/Heartbeat` on `/heartbeat` (through
`HeartbeatPublisher` from `delivery_bot_bringup`), so the stack and its health monitor run. The card replaces it
with the real node.

## Maps

`maps/city/` holds the static map of the simulated city, exported from delivery_bot_sim with
*Tools > City > Export Occupancy Map*. Don't edit it by hand: change the city in Unity, export again and replace the
files.

| File | Content |
| --- | --- |
| `map.pgm` | 2000 × 2000 cells, 8 bit: 0 = occupied, 254 = free, 205 = unknown. Image row 0 is the highest y. |
| `map.yaml` | `resolution: 0.05`, `origin: [-50.0, -50.0, 0.0]` (lower-left cell corner), `negate: 0`, `occupied_thresh: 0.65`, `free_thresh: 0.1` |
| `tiles/tile_rRR_cCC.{pgm,yaml}` | 20 m tiles of the same map that contain obstacles, for the offline benchmark corpus (delivery_bot_core#26) |

- The map frame origin is the Unity world origin, axes ROS FLU (x = Unity +Z, y = Unity −X).
- A cell is occupied if static geometry reaches 0.08–0.6 m above the ground. That band covers the LiDAR plane
  (0.515 m) and low walls, while the 0.05 m curbs stay drivable. Moving objects are never in the map.
- `free_thresh` is 0.1, not the usual 0.25, because unknown (205) gives p = 0.196 and would otherwise read as free.
- `setup.py` installs the files into `share/delivery_bot_map_server/maps/city`. `test/test_city_map.py` checks the
  format until `dbot-map validate` exists.
