# Roadmap

Each step is one small feature that works and can be tested on its own.

| # | Feature | Packages | Status |
|---|---|---|---|
| 1 | Repo structure, rover URDF with sensors, flat world, sim launch | `terrain_nav_description`, `terrain_nav_gazebo` | Done |
| 2 | Terrain world: heightmap hills, slopes, scattered rocks | `terrain_nav_gazebo` | Planned |
| 3 | Teleop + slope-driving sanity check (tilt from IMU) | `terrain_nav_gazebo` | Planned |
| 4 | EKF: wheel odom + IMU (local), then + GPS via `navsat_transform` (global) | `terrain_nav_localization` | Planned |
| 5 | Elevation map from 3D LiDAR with `grid_map` | `terrain_nav_mapping` | Planned |
| 6 | Traversability layers: slope, roughness, step height | `terrain_nav_mapping` | Planned |
| 7 | Baseline Nav2 bringup (default costmaps) | `terrain_nav_navigation` | Planned |
| 8 | Custom C++ costmap layer from traversability | `terrain_costmap_plugin` | Planned |
| 9 | Terrain-aware Nav2 config using the plugin | `terrain_nav_navigation` | Planned |
| 10 | Benchmark: default vs terrain-aware (success rate, path length, max tilt) | `terrain_nav_benchmark` | Planned |

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
