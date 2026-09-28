# Pure Pursuit Controller for ROS 2

ROS 2 Jazzy implementation of a Pure Pursuit controller for a simulated ground vehicle using Gazebo Sim.

The project includes:

- ROS 2 publisher/subscriber architecture.
- Odometry-based pose estimation.
- Quaternion-to-yaw conversion.
- Pure Pursuit steering control.
- Bicycle-model yaw-rate mapping.
- Timer-driven control loop.
- Reference trajectory recording.
- Executed trajectory recording.
- Cross-track error analysis.
- Lookahead-gain parameter tuning.
- Reproducible trajectory plots.

---

## 1. Environment

The project was developed and tested using:

- Ubuntu through WSL
- ROS 2 Jazzy
- Gazebo Sim 8.15.0
- Python 3.12
- rclpy
- nav_msgs
- geometry_msgs
- matplotlib

The simulation uses the `prius_bringup` package from the `movilidad_inteligente` repository.

---

## 2. Package Structure

pure_pursuit_controller/
├── analysis/
│   ├── analyze_trajectory.py
│   └── plot_tuning.py
├── data/
│   ├── waypoints.csv
│   ├── actual_trajectory.csv
│   ├── actual_trajectory_k05.csv
│   ├── actual_trajectory_k10.csv
│   ├── actual_trajectory_k15.csv
│   ├── trajectory_comparison.png
│   ├── cross_track_error.png
│   └── lookahead_tuning.png
├── launch/
│   └── pure_pursuit.launch.py
├── pure_pursuit_controller/
│   ├── __init__.py
│   ├── path_recorder.py
│   ├── pure_pursuit_node.py
│   └── trajectory_recorder.py
├── resource/
│   └── pure_pursuit_controller
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
└── LICENSE

---

## 3. ROS 2 Architecture

The controller uses the following ROS 2 interfaces:

                         /odom
                           |
                           v
              +-------------------------+
              |  Pure Pursuit Controller|
              |                         |
              |  Position (x,y)         |
              |  Orientation -> yaw     |
              |  Lookahead calculation  |
              |  Steering calculation   |
              |  Bicycle model          |
              +------------+------------+
                           |
                           v
                        /cmd_vel
                           |
                           v
                    Simulated vehicle

The controller subscribes to `/odom` and publishes control commands through `/cmd_vel`.

The control loop is executed using a ROS 2 timer at a configurable frequency.

---

## 4. Reference Path Recording

The reference trajectory is generated from the vehicle's odometry.

The path recorder subscribes to `/odom` and extracts the vehicle position:

x = position.x
y = position.y

A new waypoint is stored only when the vehicle has moved at least 0.05 m from the previously stored point.

The resulting reference trajectory is stored in:

data/waypoints.csv

The file contains the columns:

x,y

The final reference trajectory contains 351 points.

---

## 5. Vehicle Pose and Yaw

The vehicle orientation is received as a quaternion:

qx, qy, qz, qw

The yaw angle is calculated using:

theta =
atan2(
    2(qw qz + qx qy),
    1 - 2(qy^2 + qz^2)
)

This converts the quaternion orientation into the planar heading required by the controller.

---

## 6. Pure Pursuit Controller

The controller uses a velocity-dependent lookahead distance:

P = k v

where:

- P is the lookahead distance.
- k is the lookahead gain.
- v is the vehicle linear velocity.

A minimum lookahead distance is also implemented:

P = max(P_min, k v)

The controller searches the reference trajectory for a forward waypoint located approximately at the selected lookahead distance.

---

## 7. Steering Calculation

The heading difference between the vehicle and the selected target point is calculated as:

psi =
atan2(
    sin(theta_t - theta),
    cos(theta_t - theta)
)

where:

- theta_t is the target heading.
- theta is the vehicle heading.

The Pure Pursuit steering angle is then calculated using:

delta =
atan2(
    2 L sin(psi),
    P
)

where L is the vehicle wheelbase.

For this simulation:

L = 2.7 m

---

## 8. Bicycle Model

The steering command is converted into yaw rate using the bicycle model:

theta_dot = (v / L) tan(delta)

The resulting command is published through `/cmd_vel`.

The command uses:

linear.x  = vehicle velocity
angular.z = calculated yaw rate

An angular velocity limit is implemented to prevent excessive commands.

---

## 9. Controller Parameters

The main configurable parameters are:

Parameter                  Default value       Description
----------------------------------------------------------------------
velocity                   1.0 m/s             Vehicle linear velocity
wheelbase                  2.7 m               Vehicle wheelbase
lookahead_gain             1.0                 Gain k in P = kv
min_lookahead              0.5 m               Minimum lookahead distance
max_angular_velocity       1.5 rad/s           Maximum yaw rate
control_frequency          10 Hz               Controller update frequency
goal_tolerance             0.5 m               Goal distance tolerance

The lookahead gain is configurable through the launch system.

---

## 10. Running the Simulation

The project uses Gazebo Sim to simulate the ground vehicle.

The Pure Pursuit controller receives the simulated odometry and publishes velocity commands.

The default controller configuration uses:

v = 1.0 m/s

and:

k = 1.0

The lookahead gain can also be changed to evaluate different controller configurations.

---

## 11. Recording the Executed Trajectory

The executed trajectory is recorded from `/odom`.

The trajectory recorder stores the vehicle position during the simulation.

The resulting trajectory is stored in:

data/actual_trajectory.csv

The file contains:

x,y

Separate trajectory files were also recorded for the lookahead-gain experiments:

data/actual_trajectory_k05.csv
data/actual_trajectory_k10.csv
data/actual_trajectory_k15.csv

---

## 12. Trajectory Analysis

The analysis script calculates the minimum distance between every executed trajectory point and the reference path.

The following metrics are calculated:

- Mean cross-track error.
- Maximum cross-track error.
- Position of maximum error.
- Distance traveled when maximum error occurred.

The analysis also generates two figures:

data/trajectory_comparison.png
data/cross_track_error.png

The trajectory comparison shows the reference path and the executed vehicle trajectory.

The cross-track error plot shows the tracking error as a function of distance traveled.

---

## 13. Lookahead Gain Tuning

An experimental tuning process was performed to evaluate the effect of the lookahead gain k.

The vehicle velocity was maintained at:

v = 1.0 m/s

Therefore:

P = kv

Three lookahead gains were experimentally evaluated:

k = 0.5
k = 1.0
k = 1.5

The corresponding lookahead distances were:

Lookahead gain k    Velocity       Lookahead distance
-----------------------------------------------------
0.5                 1.0 m/s        0.5 m
1.0                 1.0 m/s        1.0 m
1.5                 1.0 m/s        1.5 m

The purpose of the experiment was to quantify how the lookahead distance affects trajectory tracking.

---

## 14. Tuning Results

The experimentally obtained cross-track errors were:

Lookahead gain k    Mean error [m]    Maximum error [m]
-------------------------------------------------------
0.5                 0.1690             0.5085
1.0                 0.0539             0.2555
1.5                 0.0077             0.0454

The corresponding trajectory recordings are stored in:

data/actual_trajectory_k05.csv
data/actual_trajectory_k10.csv
data/actual_trajectory_k15.csv

Under the tested simulation conditions, increasing the lookahead gain from 0.5 to 1.5 was associated with a reduction in both mean and maximum measured cross-track error.

For the tested trajectory and velocity, the experiment with:

k = 1.5

produced:

e_mean = 0.0077 m

and:

e_max = 0.0454 m

These results describe the tested simulation conditions and should not be interpreted as a universal optimal value for different vehicle speeds, trajectories, or environments.

---

## 15. Main Trajectory Result

The reference trajectory contains 351 points.

For the experiment using:

k = 1.5

the recorded trajectory contained 1397 points.

The resulting error analysis was:

Mean cross-track error: 0.0077 m
Maximum cross-track error: 0.0454 m
Maximum error position: (11.13, -0.16) m
Distance along trajectory at maximum error: 11.13 m

The resulting figures are stored in:

data/trajectory_comparison.png
data/cross_track_error.png

The trajectory comparison provides a visual assessment of the agreement between the reference and executed paths, while the cross-track error plot provides a quantitative representation of tracking performance.

---

## 16. Reproducing the Tuning Experiment

The tuning experiment was performed independently for each value of k.

For each experiment:

1. The trajectory recorder was started and subscribed to `/odom`.
2. The Pure Pursuit controller was launched with the selected lookahead gain.
3. The vehicle followed the reference trajectory.
4. The simulation was allowed to continue until the goal tolerance was reached.
5. The executed trajectory was saved as a CSV file.
6. The resulting trajectory was analyzed using the same cross-track error calculation.

The three experimental configurations were:

k = 0.5
k = 1.0
k = 1.5

The resulting trajectories were stored separately to allow direct comparison between experiments.

---

## 17. Plotting the Tuning Results

The tuning results are summarized in:

data/lookahead_tuning.png

The figure compares the mean and maximum cross-track errors obtained for the three tested lookahead gains.

The plot was generated programmatically from the experimentally measured values to make the analysis reproducible.

---

## 18. ROS 2 Tests

The package includes ROS 2 Python package tests for:

- Copyright headers.
- PEP 8 style.
- PEP 257 documentation conventions.

The package passed the implemented automated tests with:

3 tests
0 errors
0 failures
1 skipped

The skipped test corresponds to the package's standard ROS 2 test configuration.

---

## 19. Build

The package was successfully built using the ROS 2 Jazzy build system.

The package is an `ament_python` ROS 2 package and uses:

- setuptools
- rclpy
- nav_msgs
- geometry_msgs
- ament_index_python

The package can be rebuilt using the ROS 2 colcon build system.

---

## 20. Git Repository

The project is version controlled using Git.

The repository is hosted on GitHub:

https://github.com/RataCuco25/pure_pursuit_controller

The repository contains:

- ROS 2 controller implementation.
- Reference path recorder.
- Executed trajectory recorder.
- Gazebo launch configuration.
- Trajectory analysis scripts.
- Lookahead tuning script.
- Experimental trajectory data.
- Generated analysis plots.
- Documentation.

---

## 21. Project Scope

This project focuses on the implementation and evaluation of a Pure Pursuit path-tracking controller for a simulated ground vehicle.

The main objectives were:

1. Implement ROS 2 publisher/subscriber communication.
2. Obtain vehicle pose from odometry.
3. Convert quaternion orientation into yaw.
4. Implement velocity-dependent Pure Pursuit lookahead.
5. Calculate the steering command.
6. Convert steering to yaw rate using the bicycle model.
7. Execute the controller using a timer-based loop.
8. Record a reference trajectory.
9. Record the executed trajectory.
10. Quantify trajectory tracking error.
11. Experimentally evaluate different lookahead gains.
12. Generate reproducible trajectory and tuning plots.

The controller was evaluated using a simulated ground vehicle in Gazebo Sim under the tested trajectory and velocity conditions.
