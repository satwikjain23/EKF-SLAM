
# EKF-SLAM

This repository contains the EKF-SLAM implementation integrated with the F1TENTH simulator. The system performs perception, cone detection, SLAM, and control using multiple ROS nodes within the race package.

Developed and tested on ROS Noetic (Ubuntu 20.04).

![ekf](./ekf.gif)

## Installation

Clone this repository into your ROS workspace and build it:

```bash
cd ~/catkin_ws/src
git clone https://github.com/satwikjain23/EKF-SLAM.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash

```
## Run

* Launch the simulator
```
rosrun f1tenth_simulator simulator.launch
```
<br>

* Run the Perception Node

```
rosrun race perception.py
```
<br>

* Run the Covariance Ellipse Visualization Node

```
rosrun race conemarker.py
```
> ADD /marker_visualization IN RVIZ BY TOPIC
<br>

* Run the Ground Truth Visualization Node

```
rosrun race marker.py
```
> ADD /visualization_marker IN RVIZ BY TOPIC
<br>

* Run the EKF Path Visualization Node

```
rosrun race new_marker.py
```
> ADD /dynamic_viz IN RVIZ BY TOPIC
<br>

* Run the EKF Node

```
rosrun race ekf.py
```
<br>

* Run the Control Node

```
rosrun race cones_control.py
```

<br>

* Run the Path Planning Node

```
rosrun race cones_distfinder.py
```

<br>

* Run the Cone Pair Matcher Node

```
rosrun race slam.py
```

    
