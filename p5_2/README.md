# Autonomous Home Service Robot (ROS)

An indoor mobile robot built on ROS that carries out autonomous pick-up and delivery tasks. It constructs a map of its environment on the fly, localizes itself within that map, and plans collision-free paths to reach target locations. Object interactions are simulated through RViz markers that appear and disappear at the appropriate moments.

---

## System Overview

The robot combines three core capabilities:

- **Mapping** – builds a 2D floor plan while being driven around
- **Localization** – tracks its own position within the saved map
- **Autonomous Navigation** – plans and follows paths to goal coordinates

---

## Dependencies & Packages

| Package | Category | Role |
|---|---|---|
| `slam_gmapping` | Mapping | Implements the GMapping SLAM algorithm using a Rao-Blackwellized particle filter to build a 2D occupancy grid from laser scan and odometry data as the robot is manually driven through the environment |
| `turtlebot_navigation` | Navigation | Provides the `move_base` navigation stack with global and local planners for autonomous path planning and obstacle avoidance |
| `turtlebot_simulator` | Simulation | Supplies Gazebo launch files and configuration for spawning the TurtleBot in a simulated environment with physics, sensors, and actuators |
| `kobuki_gazebo_plugins` | Simulation | Gazebo plugins that simulate the Kobuki base's wheel controllers, cliff sensors, and bumpers, enabling realistic physics-based testing in simulation |
| `turtlebot_interactions` | Visualization | RViz configurations and interactive tools for monitoring and controlling the TurtleBot during operation |
| `turtlebot` | Robot Platform | Core TurtleBot meta-package covering robot description, bringup scripts, and configuration for the Kobuki mobile base |
| `kobuki_description` | Robot Platform | URDF/Xacro model of the Kobuki base, defining its physical geometry, joint structure, and sensor mount points |
| `yocs_cmd_vel_mux` | Velocity Multiplexing | Manages and prioritizes multiple simultaneous velocity command sources (e.g. teleoperation vs. autonomous navigation), ensuring only one source controls the robot at a time |

---
## Custom Packages

### `pick_objects`
Sends a sequence of navigation goals to the `move_base` action server. The robot travels to a designated pickup coordinate, pauses for 5 seconds, then proceeds to the drop-off coordinate.

### `add_markers`
Publishes RViz `Marker` messages to simulate physical object handling. The `add_markers_service` variant subscribes to `/amcl_pose` and uses a state machine to show or hide a marker cube depending on how close the robot is to each zone.

### `my_robot`
Holds environment-specific assets: the Gazebo world file, saved map files (`map.pgm` / `map.yaml`), AMCL launch configuration, and navigation costmap parameters.

---

## Getting Started

### 1. Build the Workspace

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

---

### 2. SLAM Mapping

Drive the robot through the environment to generate a map:

```bash
./scripts/test_slam.sh
```

This starts Gazebo (project world), GMapping, RViz, and keyboard teleoperation simultaneously. Steer the robot through every area of the space and watch the occupancy grid fill in within RViz.

Once coverage looks complete, save the map from a new terminal:

```bash
source devel/setup.bash
rosrun map_server map_saver -f $(pwd)/src/my_robot/maps/map
```

Two files will be written — `map.pgm` and `map.yaml` — inside `src/my_robot/maps/`. All subsequent scripts rely on these files for localization and navigation.

> **Expected result:** Gazebo opens with the indoor scene. Driving the robot progressively fills the RViz occupancy grid. After saving, a complete map is available for AMCL.

---

### 3. Navigation Test

Confirm that the robot can autonomously reach manually specified goals:

```bash
./scripts/test_navigation.sh
```

> **Expected result:** RViz loads with the saved map. The robot localizes via AMCL. Use the *2D Nav Goal* tool in RViz to send target poses — the robot plans and follows a path to each one.

---

### 4. Autonomous Pick & Drop

Validate the two-waypoint navigation sequence:

```bash
./scripts/pick_objects.sh
```

> **Expected result:** The robot drives to the pickup zone (−1.0, 2.5), waits 5 seconds, then drives to the drop-off zone (0.0, 0.0).

---

### 5. Marker Visualization

Test the RViz marker logic in isolation:

```bash
./scripts/add_markers.sh
```

> **Expected result:** A green cube appears at the pickup zone, vanishes after 5 seconds (pickup simulated), then reappears at the drop-off zone after another 5 seconds.

---

### 6. Full Home Service Demo

Run the complete end-to-end workflow:

```bash
./scripts/home_service.sh
```

> **Expected result:** The robot navigates to the pickup zone while a marker is displayed there. On arrival the marker disappears (object collected) and the robot waits 5 seconds before heading to the drop-off zone, where the marker reappears (object delivered). The entire pick-up-and-delivery cycle completes without any human intervention.
