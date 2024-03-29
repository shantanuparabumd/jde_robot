# Autonomous Robot Navigation using NAV2



## Setup

Create a workpace

```sh
mkdir -p jde_ws/src
cd ~/jde_ws/src
```

Clone the reposiory

```sh
git clone https://github.com/shantanuparabumd/turtlebot3_project3.git
```

Source ROS (Enable ROS commands)

```sh
source /opt/ros/galactic/setup.bash
```

Build the workspace

```sh
cd ~/jde_ws
colcon build --packages-select jde_robot
```


Source ROS (Package will be identified)

```sh
source install/setup.bash
```

## Part 1: A

Open a new terminal and source the ROS Environemnt as above.
Run the below command to start the publsiher

```sh
ros2 run jde_robot publisher_node.py
```

Open a new terminal and source the ROS Environemnt as above.
Run the below command to start the subscriber.
```sh
ros2 run jde_robot subscriber_node.py
```

Explore Topics

```sh
ros2 topic list
```


## Part 1: B


Use the below command to simultaneously launch Gazebo and RVIZ with appropriate configuration to visualize the Laser.
```sh
ros2 launch jde_robot rviz_laser.launch.py
```

You can run the teleop script to move the robot around to see the laser data change.

```sh
ros2 run jde_robot teleop.py
```

## Part 2:

Use the below command to launch all the neccessary nodes.
```sh
ros2 launch jde_robot jde_robot.launch.py
```

This will start Gazebo, RVIZ and the NAV2 navigation and localization launch files.
The map and rviz are pre configured.

It will take 25 seconds for everything to setup and then the robot will start moving to 3 waypoints around the map.


## Demonstration