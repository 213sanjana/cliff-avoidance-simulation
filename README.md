# Edge Avoiding Robot

A ROS 2 simulation of differential-drive robots navigating an elevated table environment. The project includes a single-robot demonstration and a two-robot demonstration using the same robot model and edge-detection behavior.

The robots use simulated laser scans to detect changes in the surface ahead. When the edge-detection thresholds are reached, the controller commands reverse motion and a turn before continuing forward.

## Single Edge Avoiding Robot

The single-robot launch starts Gazebo, spawns one robot in the table world, loads its wheel controllers, and runs the edge-avoidance node.

```bash
ros2 launch bot_bringup simulated_robot.launch.py
```

The default world is `empty_table`. To select another included world:

```bash
ros2 launch bot_bringup simulated_robot.launch.py world_name:=empty
```

The robot receives laser data on `/scan` and publishes motion commands on `/wheel_controller/cmd_vel_unstamped`. RViz is not started by this launch.

## Two Edge Avoiding Robots

The two-robot launch creates two instances named `robot1` and `robot2` on the existing table layout. Each robot has its own laser topic, controller manager, coordinate frames, and edge-avoidance node. The robots are displayed in red and blue.

```bash
ros2 launch bot_bringup two_robot_demo.launch.py
```

A shared collision guard uses simulator model poses to filter motion commands, maintain separation, and constrain recovery motion using the platform boundaries. Each robot continues to run its own edge-avoidance behavior.

Run without the Gazebo graphical interface:

```bash
ros2 launch bot_bringup two_robot_demo.launch.py gui:=false
```

Disable autonomous edge avoidance for manual command input:

```bash
ros2 launch bot_bringup two_robot_demo.launch.py edge_avoidance:=false
```

Manual commands go to `/robot1/cmd_vel` and `/robot2/cmd_vel`. The collision guard forwards filtered commands on each robot's `safe_cmd_vel` topic. Laser scans are available on `/robot1/scan` and `/robot2/scan`.

## Environment

The project targets Ubuntu 22.04, ROS 2 Humble, and Gazebo Fortress through `ros_gz_sim`. It also uses `ign_ros2_control`, ROS 2 controllers, Python 3, Xacro, and colcon. A graphical desktop is needed for the default Gazebo interface.

## Build Locally

Open a terminal in the repository root. The four ROS packages live directly in this directory; a separate `src` folder is not required.

With ROS 2 Humble and rosdep already installed and initialized:

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths bot_bringup bot_controller bot_description bot_script --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

Source ROS and the workspace in each additional terminal before running launch commands:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Packages

- `bot_description`: robot geometry, sensors, Gazebo integration, worlds, table assets, and RViz configuration.
- `bot_controller`: wheel-controller configuration and controller launch.
- `bot_script`: laser-based edge avoidance and the two-robot collision guard.
- `bot_bringup`: single-robot and two-robot launch files.

## Validation

The collision guard includes unit tests:

```bash
PYTHONPATH="$PWD/bot_script${PYTHONPATH:+:$PYTHONPATH}" python3 -m pytest bot_script/test/test_collision_guard.py
```

To run the package test suites in a configured ROS environment:

```bash
colcon test
colcon test-result --verbose
```

Simulation behavior depends on sensor placement, scan values, platform geometry, and controller settings. This project demonstrates simulated behavior; physical hardware requires its own validation.

## Troubleshooting

- If a package cannot be found, source both ROS and `install/setup.bash` in the current terminal.
- If dependencies are missing, repeat the rosdep installation command from the repository root and rebuild.
- If models do not appear, check that `bot_description` built successfully and that the workspace is sourced.
- For a two-robot run without a desktop session, set `gui:=false`.
- Stop a running launch with `Ctrl+C` before starting another simulation that uses the same topics and model names.

## Repository Maintenance

Build outputs, installation files, logs, Python caches, local environment files, and editor settings are excluded through `.gitignore`. Package contact fields use `maintainers@example.com` as a placeholder; replace it with a real project contact when assigning maintainership.
