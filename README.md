# Complete Navigation Pipeline

![image](https://github.com/KAN201197/Complete_Navigation_Pipeline/assets/128454220/43bfb0cc-0e97-4e30-84c1-ee955a0ef0a4)

In this project, we implement a complete navigation pipeline which consist of perception (object detection and recognition using YOLOv8), 2D & 3D SLAM, Localization using AMCL, and Global path planner & Local path planner using move_base node. From this navigation, we also implement exploration strategy while locate and detect desire object (box) in the Environment. After the robot able to detect the object, the robot will approach to target object based on depth distance estimation from depth camera. In the robot configuration, the robot use multi-sensor obstacle detection using 2 Lidars and 1 Kinect depth camera. The robot configuration can be seen from below image.

![image](https://github.com/KAN201197/Complete_Navigation_Pipeline/assets/128454220/03df278c-ac1d-4276-a56b-905a8cc46fad)

In this project also we set non-passable area which we use a seperate map server for sending the traversable map to the planner (one map used for localization and other map used for motion planning). The object detection is done by kinect camera which already transform into 2D lidar data and rear 2D lidar to do obstacle avoidance. The capability of obstacle avoidance can be seen from image below.

![DWAObstacleAvoidance-ezgif com-resize](https://github.com/KAN201197/Complete_Navigation_Pipeline/assets/128454220/56a1d68b-80e0-4498-896a-603837ce31c8)

## Dependencies

* System Requirements:
  * Ubuntu 20.04 (18.04 not yet tested)
  * ROS Noetic (Melodic not yet tested)
  * C++11 and above
  * CMake: 3.0.2 and above
* This repo depends on the following standard ROS pkgs:
  * `roscpp`
  * `rospy`
  * `rviz`
  * `std_msgs`
  * `nav_msgs`
  * `geometry_msgs`
  * `visualization_msgs`
  * `tf`
  * `tf2`
  * `tf2_ros`
  * `tf2_geometry_msgs`
  * `pluginlib`
  * `map_server`
  * `gazebo_ros`
  * `jsk_rviz_plugins`
  * `jackal_gazebo`
  * `jackal_navigation`
  * `velodyne_simulator`
  * `teleop_twist_keyboard`
* And this [gazebo_model](https://github.com/osrf/gazebo_models) repository

## Installation

This repo is a ros workspace, containing eight rospkgs:

* `me5413_world` the main pkg containing the gazebo world, and the launch files
* `me5413_perception` pkg for perception part and exploration part of project
* `me5413_mapping` pkg for hector SLAM
* `velodyne_simulator` velodyne LiDAR with GPU acceloration
* `laserscan_merger` merge front 2D LiDAR and rear 2D LiDAR
* `interactive_tools` are customized tools to interact with gazebo and your robot
* `jackal_description` contains the modified jackal robot model descriptions
* `A-LOAM` pkg for A-LOAM SLAM

The maps can be found at `src/me5413_world/maps`.

```bash

mkdir catkin_ws

cd catkin_ws

git https://github.com/KAN201197/Complete_Navigation_Pipeline.git

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r requirements.txt

# Build
catkin_make

# Source
source devel/setup.bash
```

To properly load the gazebo world, you will need to have the necessary model files in the `~/.gazebo/models/` directory.

There are two sources of models needed:

* [Gazebo official models](https://github.com/osrf/gazebo_models)

  ```bash
  # Create the destination directory
  cd
  mkdir -p .gazebo/models

  # Clone the official gazebo models repo (assuming home here `~/`)
  git clone https://github.com/osrf/gazebo_models.git

  # Copy the models into the `~/.gazebo/models` directory
  cp -r ~/gazebo_models/* ~/.gazebo/models
  ```

* [Our customized models](https://github.com/NUS-Advanced-Robotics-Centre/ME5413_Final_Project/tree/main/src/me5413_world/models)

  ```bash
  # Copy the customized models into the `~/.gazebo/models` directory
  cp -r ~/ME5413_Final_Project/src/me5413_world/models/* ~/.gazebo/models
  ```

## Usage

### 1. Launch Gazebo world

```bash
# Launch Gazebo World together with our robot
roslaunch me5413_world world.launch
```

### 2. Launch Navigation
After launching step 1 in a separate terminal

Launches Navigation for the robot

```bash
roslaunch me5413_world navigation.launch
```
Can choose which production line to go to in Rviz by clicking simple panel.

![image](https://github.com/KAN201197/Complete_Navigation_Pipeline/assets/128454220/c5ee9003-8462-40ea-9ff4-3296f2c721ab)

### 3. Perception & Exploration
After launching step 2 in a separate terminal

Lauch perception and exploration for the Robot

```bash
roslaunch me5413_perception exploration.launch
```
When the robot find the target object, it will adjust the global path which generate a new path to the target object which can be seen below.

![image](https://github.com/KAN201197/Complete_Navigation_Pipeline/assets/128454220/02678966-9eb0-485b-acab-d872331dac12)
