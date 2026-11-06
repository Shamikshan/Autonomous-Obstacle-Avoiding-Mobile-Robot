# Autonomous Obstacle-Avoiding Mobile Robot

![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-22314E?logo=ros&logoColor=white)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic-FF6F00?logo=gazebo&logoColor=white)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu&logoColor=white)
![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)

A differential-drive mobile robot simulation built with **ROS  2 Jazzy**, **Gazebo Harmonic (Gazebo Sim 8)**, and **Ubuntu 24.04**. The project demonstrates autonomous navigation, 2D LiDAR-based mapping, obstacle avoidance, and robot coordinate-frame management.

## Features

- **SLAM:** Builds a 2D occupancy-grid map of an initially unknown environment using SLAM Toolbox.
- **Autonomous navigation:** Uses Nav2 with the Navfn (A*) planner and Regulated Pure Pursuit controller.
- **Waypoint patrol:** A Python node sends navigation goals and monitors the robot pose through TF2.
- **Reactive obstacle avoidance:** A separate Python node divides the LiDAR scan into front, front-left, and front-right sectors and publishes velocity commands directly.
- **Simulation:** Runs a differential-drive robot equipped with a 360° GPU LiDAR sensor in a cluttered 10 m × 10 m Gazebo world.

## Project Overview & Demos

### Mode A: Full Nav2 Autonomous Stack + SLAM + Waypoint Patrol
The robot builds an occupancy grid map in an unknown 10m &times; 10m cluttered arena using SLAM Toolbox while Nav2 handles global path planning (Navfn A\*) and path tracking (Regulated Pure Pursuit). A custom Python commander node dispatches autonomous patrol waypoints and monitors real-time TF2 poses.

#### Video of a navigation goal, selected **2D Goal Pose** in RViz2 and click-drag on the map.
https://github.com/user-attachments/assets/2bf75a7a-48eb-4c84-834d-fb8159f479ea

#### Video of autonomous waypoint patrol
https://github.com/user-attachments/assets/cc1b0be6-b223-41c7-9857-775d40f19be7

---

### Mode B: Standalone Reactive Python LIDAR Obstacle Avoidance
A standalone Python node (`lidar_obstacle_avoidance.py`) that subscribes to `/scan`, slices the 360-degree field of view into **Front**, **Front-Left**, and **Front-Right** sectors, filters invalid measurements, and commands `/cmd_vel` directly without relying on costmaps or Nav2.

https://github.com/user-attachments/assets/db1abc74-dc5b-4770-a5c3-68646f3ea354

<!-- ============================================================= -->

---

## System Architecture

```text
                 Gazebo Harmonic
          (Differential Drive + GPU LiDAR)
                         |
                         v
                    ros_gz_bridge
          (/clock, /cmd_vel, /odom, /scan, /tf)
                         |
          +--------------+----------------+
          |              |                |
          v              v                v
 robot_state_publisher  SLAM Toolbox     Nav2
   (robot TF frames)   (map -> odom)   (planning,
                                         control,
                                         recovery)
          |                               |
          +---------------+---------------+
                          |
                         RViz2

Reactive mode:
Gazebo LiDAR -> /scan -> lidar_obstacle_avoidance.py
                            |
                            v
                         /cmd_vel
```

## TF Frame Tree

```text
map
 └── odom                      
      └── base_footprint       
           └── base_link       
                ├── caster_wheel
                ├── left_wheel
                ├── right_wheel
                └── lidar_link
```

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `colcon`
- The ROS 2 packages listed below

Install the required packages:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-simple-commander \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-rviz2
```

## Installation

Create a ROS 2 workspace, clone this repository into `src`, and build the package:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Shamikshan/Autonomous-Obstacle-Avoiding-Mobile-Robot.git
cd ~/ros2_ws

colcon build --symlink-install
source install/setup.bash
```


## Run the Project

### Mode A — SLAM, Nav2, and RViz2

Start the simulation in **Terminal 1**:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch auto_nav_bot sim.launch.py
```

Start navigation in **Terminal 2**:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch auto_nav_bot navigation.launch.py
```

To set a navigation goal, select **2D Goal Pose** in RViz2 and click-drag on the map.

To start autonomous waypoint patrol, use **Terminal 3**:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run auto_nav_bot autonomous_patrol --ros-args -p use_sim_time:=true
```

### Mode B — Reactive LiDAR Obstacle Avoidance

Start the simulation:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch auto_nav_bot sim.launch.py
```

In another terminal, run the reactive avoidance node:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run auto_nav_bot lidar_avoidance --ros-args -p drive_mode:=true
```

## Repository Structure

```text
auto_nav_bot/
├── auto_nav_bot/
│   ├── autonomous_patrol.py
│   └── lidar_obstacle_avoidance.py
├── config/
│   ├── gz_bridge.yaml
│   ├── nav2_params.yaml
│   └── slam_params.yaml
├── docs/
│   └── images/
├── launch/
│   ├── navigation.launch.py
│   └── sim.launch.py
├── rviz/
│   └── nav2_view.rviz
├── urdf/
│   └── robot.urdf
├── worlds/
│   └── cluttered_world.sdf
├── package.xml
├── setup.cfg
└── setup.py
```


## Nav2 Bringup: Challenges & Fixes

| # | Problem | Cause | Fix |
|---|---------|-------|-----|
| 1 | `controller_server` and `lifecycle_manager` crashed with exit code 127 | ABI mismatch / undefined symbols in `diagnostic_updater` and Fast-CDR (`libnav2_msgs__rosidl_typesupport_fastrtps_cpp.so`) | Upgraded and synced `libfastcdr`, `ros-jazzy-fastrtps`, `ros-jazzy-diagnostic-updater`, `ros-jazzy-nav2-msgs` |
| 2 | `InvalidParameterTypeException` on `local_costmap` | Jazzy strictly validates types; `width`/`height` were doubles (`3.0`) but must be integers | Changed to `width: 3` and `height: 3` in `nav2_params.yaml` |
| 3 | `Couldn't load critics! No critics defined for FollowPath` | Default `DWBLocalPlanner` needs trajectory critics, none were defined | Set plugin to `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` |
| 4 | Nodes stuck in `inactive [2]` / `unconfigured [1]`, then self-terminated | Generic bringup also managed `docking_server` and `collision_monitor` (incomplete config). One failure triggers `Aborting bringup`, and the bond heartbeat then times out | Rewrote `launch/navigation.launch.py` to manage only 4 nodes (`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`) with a dedicated `lifecycle_manager` and `bond_timeout: 0.0` |


## License

Distributed under the Apache 2.0 License. See `package.xml` for details.

____________________________________________