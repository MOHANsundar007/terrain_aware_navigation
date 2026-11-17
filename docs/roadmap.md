# Roadmap

Each step is one small feature that works and can be tested on its own.

| # | Feature | Packages | Status |
|---|---|---|---|
| 1 | Repo structure, rover URDF with sensors, flat world, sim launch | `terrain_nav_description`, `terrain_nav_gazebo` | Done |
| 2 | Terrain world: generated mesh with rocks, ditch, ridge, hill | `terrain_nav_gazebo` | Done |
| 3 | EKF: wheel odom + IMU (local), then + GPS via `navsat_transform` (global) | `terrain_nav_localization` | Done |
| 4 | Elevation map from 3D LiDAR with `grid_map` | `terrain_nav_mapping` | Done |
| 5 | Traversability layers: slope, roughness, step height | `terrain_nav_mapping` | Done |
| 6 | Baseline Nav2 bringup (default costmaps) | `terrain_nav_navigation` | Planned |
| 7 | Custom C++ costmap layer from traversability | `terrain_costmap_plugin` | Planned |
| 8 | Terrain-aware Nav2 config using the plugin | `terrain_nav_navigation` | Planned |
| 9 | Benchmark: default vs terrain-aware (success rate, path length, max tilt) | `terrain_nav_benchmark` | Planned |

## Day 1 verification

Run headless and check that every sensor is publishing:

```bash
ros2 launch terrain_nav_gazebo sim.launch.py headless:=true rviz:=false
ros2 topic hz /lidar/points   # ~10 Hz
ros2 topic hz /imu/data       # ~100 Hz
ros2 topic hz /gps/fix        # ~5 Hz
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
ros2 topic echo /odom --field pose.pose.position   # x increases
```
