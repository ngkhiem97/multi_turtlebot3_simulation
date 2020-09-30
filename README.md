# multi_turtlebot3_environment

Environment for Multi Turtlebot3 System

## Clone Repository to catkin workspace
```bash
cd ~/catkin_ws/src 
git clone https://github.com/ngkhiem97/multi_turtlebot3_environment.git
cd ~/catkin_ws
catkin_make
```

## Launching Gazebo environment with 3 robot on World View

```bash
roslaunch multi_turtlebot3_environment multi_turtlebot3_world.launch
```

## Launch Navigation for 1 robot with Rviz UI

```bash
roslaunch multi_turtlebot3_environment multi_turtlebot3_navigation.launch
```
