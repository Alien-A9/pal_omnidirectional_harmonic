# PAL Omnidirectional Base — Gazebo Harmonic (ROS 2 Jazzy)

## Prerequisites (Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic)
Ensure the ROS-Gazebo Harmonic integration packages are installed:
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge python3-numpy python3-matplotlib

```

## 1. Build the Workspace

From the `pal_omnidirectional_harmonic` workspace root:

```bash
colcon build --symlink-install
source install/setup.bash

```

## 2. Launch the Simulation

Terminal 1 — Launch Gazebo Harmonic and the ROS-Gazebo bridge:

```bash
source install/setup.bash
ros2 launch pal_omni_harmonic sim.launch.py

```

## 3. Run the Controllers / Spawner

* **Manual Omnidirectional Teleop Controller:**
```bash
source install/setup.bash
ros2 run pal_omni_harmonic omni_controller

```


* **Spawn a Cylindrical Obstacle (`<ox>,<oy>,<radius>`):**
```bash
source install/setup.bash
ros2 run pal_omni_harmonic spawn_obstacle 1.5,1.5,0.3

```


* **Safe Controller (PID + CBF-QP Obstacle Avoidance):**
```bash
source install/setup.bash
ros2 run pal_omni_harmonic safe_controller 3.0 3.0

```
