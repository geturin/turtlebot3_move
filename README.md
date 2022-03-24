# Random move plugin for turtlebot3

A ROS package can let turtlebot3 keep moving avoiding obstacles and go to random goal point in the standard world set（gazebo）

video：https://youtu.be/ilukuZFjg7M

base on ROS noetic,Ubuntu 20.04 and python3
# Installation

## 1. ROS Navigation

```
sudo apt-get install ros-noetic-navigation*
```

## 2. turtlebot3_gazebo
[in your work space]
```
cd ~/catkin_ws/src

git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_gazebo_plugin.git

cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```
and set turtlebot3 model
```
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## 3.turtlebot3_move
put this package in your work space
```
cp -r turtlebot3_move ~/catkin_ws/src
```
or
```
cd ~/catkin_ws/src
git clone https://github.com/geturin/turtlebot3_move.git
```
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

# How to run
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_move move.launch
```
