# Terrain-Aware Outdoor Navigation

A ROS 2 project for navigating a 4-wheel rover over uneven outdoor terrain.
The robot builds an elevation map from 3D LiDAR, turns it into a
traversability map (slope, roughness, step height) and feeds that into a
custom Nav2 costmap layer, so it avoids steep hills and rocks that a flat
2D costmap cannot see.

| | |
|---|---|
| ROS distro | ROS 2 Humble |
| Simulator | Gazebo Fortress (Ignition Gazebo 6) via `ros_gz` |
| Navigation | Nav2 |
| Localization | `robot_localization` EKF (wheel odom + IMU + GPS) |
| Mapping | `grid_map` elevation and traversability layers |

## Repository layout

```
terrain_aware_navigation/
├── terrain_nav_description/     # Robot model
│   ├── urdf/
│   │   ├── rover.urdf.xacro     # Top-level model
│   │   ├── properties.xacro     # Dimensions, masses, inertia macros
│   │   ├── chassis.xacro        # base_footprint + base_link
│   │   ├── wheel.xacro          # Wheel + continuous joint macro
│   │   ├── sensors.xacro        # 3D LiDAR, IMU, GPS (+ Gazebo sensors)
│   │   └── gazebo.xacro         # Skid-steer drive + joint state plugins
│   ├── launch/display.launch.py # Model in RViz only
│   └── rviz/rover.rviz
├── terrain_nav_gazebo/          # Simulation
│   ├── worlds/empty_terrain.sdf # Flat, geo-referenced baseline world
│   ├── config/bridge.yaml       # ROS <-> Gazebo topic bridge
│   └── launch/sim.launch.py     # Gazebo + robot + bridge + RViz
└── docs/roadmap.md
```

## Robot

4-wheel skid-steer rover, 0.8 m x 0.5 m chassis, 0.15 m wheels, about 27 kg.

| Sensor | Gazebo type | ROS topic | Message | Rate |
|---|---|---|---|---|
| 3D LiDAR (16 beams, ±15°, 30 m) | `gpu_lidar` | `/lidar/points` | `sensor_msgs/PointCloud2` | 10 Hz |
| | | `/lidar/scan` | `sensor_msgs/LaserScan` | 10 Hz |
| IMU | `imu` | `/imu/data` | `sensor_msgs/Imu` | 100 Hz |
| GPS | `navsat` | `/gps/fix` | `sensor_msgs/NavSatFix` | 5 Hz |

Other topics: `/cmd_vel` (in), `/odom`, `/joint_states`, `/tf`, `/clock`.

TF tree: `odom -> base_footprint -> base_link -> {wheels, lidar_link, imu_link, gps_link}`.
For now the Gazebo drive plugin publishes `odom -> base_footprint`. That
transform will move to the EKF once localization is added.

## Build

```bash
mkdir -p ~/terrain_nav_ws/src && cd ~/terrain_nav_ws/src
git clone <this-repo-url> terrain_aware_navigation
cd ~/terrain_nav_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Gazebo, the robot and RViz:

```bash
ros2 launch terrain_nav_gazebo sim.launch.py
```

Launch arguments: `world:=<abs path>`, `headless:=true`, `rviz:=false`,
`x:= y:= z:= yaw:=` (spawn pose).

Drive it:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Model only (no simulator):

```bash
ros2 launch terrain_nav_description display.launch.py
```

## Testing

```bash
colcon test --packages-select terrain_nav_description && colcon test-result --verbose
```

This expands the xacro and checks the result with `check_urdf`.

## Progress

See [docs/roadmap.md](docs/roadmap.md).

- [x] **Day 1**: Repo structure, robot URDF (LiDAR/IMU/GPS), baseline world, simulation launch
