# Pure Pursuit Controller

ROS 2 Jazzy package for trajectory recording, Pure Pursuit path following, and trajectory analysis using a simulated ground vehicle in Gazebo Sim.

## Description

This package implements a Pure Pursuit controller for a simulated ground vehicle. The controller receives the vehicle pose through `/odom` and publishes velocity commands through `/cmd_vel`.

The package also includes tools to:

* Record reference trajectories from `/odom`.
* Follow a predefined trajectory using Pure Pursuit.
* Record the executed trajectory.
* Calculate cross-track error.
* Generate trajectory comparison and error plots.

## Requirements

* Ubuntu / WSL
* ROS 2 Jazzy
* Gazebo Sim 8
* Python 3
* NumPy
* Matplotlib

The simulation uses the `prius_bringup` package from the `movilidad_inteligente` repository.

## Package structure

```text
pure_pursuit_controller/
├── analysis/
│   └── analyze_trajectory.py
├── launch/
│   └── pure_pursuit.launch.py
├── pure_pursuit_controller/
│   ├── path_recorder.py
│   ├── pure_pursuit_node.py
│   └── trajectory_recorder.py
├── resource/
├── test/
├── package.xml
├── setup.py
└── README.md
```

## ROS 2 interfaces

### Subscribed topic

`/odom`

Message type:

```text
nav_msgs/msg/Odometry
```

The controller uses the vehicle position and orientation from the odometry message.

### Published topic

`/cmd_vel`

Message type:

```text
geometry_msgs/msg/Twist
```

The controller publishes the vehicle linear velocity and angular velocity.

## Pure Pursuit controller

The controller uses a dynamic lookahead distance:

$$
P = k|v|
$$

The steering command is calculated using:

$$
\delta =
\operatorname{atan2}
\left(
2L\sin(\psi),
P
\right)
$$

where:

* \(L\) is the vehicle wheelbase.
* \(P\) is the lookahead distance.
* \(\psi\) is the heading error.
* \(k\) is the lookahead gain.

For the simulated vehicle, the steering command is converted to angular velocity using the bicycle model:

$$
\dot{\theta}
=
\frac{v}{L}\tan(\delta)
$$

The controller runs using a timer-based control loop.

## Main parameters

| Parameter              | Default value | Description                |
| ---------------------- | ------------: | -------------------------- |
| `velocity`             |       1.0 m/s | Reference vehicle velocity |
| `wheelbase`            |         2.7 m | Vehicle wheelbase          |
| `lookahead_gain`       |           1.0 | Dynamic lookahead gain     |
| `min_lookahead`        |         0.5 m | Minimum lookahead distance |
| `max_angular_velocity` |     1.5 rad/s | Angular velocity limit     |
| `control_frequency`    |         10 Hz | Controller frequency       |
| `goal_tolerance`       |         0.5 m | Final position tolerance   |

## Running the simulation

Source ROS 2 and the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Launch Gazebo and the Pure Pursuit controller:

```bash
ros2 launch pure_pursuit_controller pure_pursuit.launch.py
```

The launch file starts the Prius Gazebo simulation through `prius_bringup` and starts the Pure Pursuit controller.

## Recording a reference trajectory

Run the simulation and execute:

```bash
ros2 run pure_pursuit_controller path_recorder
```

The recorder subscribes to `/odom` and stores a point whenever the vehicle has moved at least 0.05 m.

The resulting trajectory is saved as:

```text
~/ros2_ws/data/waypoints.csv
```

## Recording the executed trajectory

The actual vehicle trajectory can be recorded using:

```bash
ros2 run pure_pursuit_controller trajectory_recorder
```

The resulting file is:

```text
~/ros2_ws/data/actual_trajectory.csv
```

## Trajectory analysis

After recording both trajectories, run:

```bash
python3 src/pure_pursuit_controller/analysis/analyze_trajectory.py
```

The analysis calculates the mean and maximum cross-track error and generates:

```text
data/trajectory_comparison.png
data/cross_track_error.png
```

For the evaluated trajectory, the measured values were:

* Mean cross-track error: 0.0539 m
* Maximum cross-track error: 0.2555 m

The maximum error occurred near the final curved section of the trajectory.

## Results

The controller successfully followed the reference trajectory and stopped when the vehicle reached the final waypoint.

The executed trajectory remained close to the reference trajectory over most of the path. The largest deviation occurred in the curved section, where the maximum measured cross-track error was 0.2555 m.

## Author

Pure Pursuit controller developed as part of a ROS 2 autonomous vehicle simulation project.
