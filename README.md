# ROS2_tf2_Broadcaster
Using Rviz Marker Messages for Visualizing  tf2_ros/transform_broadcaster

**System Info:** These packages were tested on Ubuntu 22.04 with ROS Humble.

### Runtime Instructions

Open Terminal 
```
mkdir ros2_ws
cd ros2_ws && mkdir src
cd src
```
After cloning the repo and building it, just type `ros2 launch rviz_marker display_marker.launch.py` into the terminal to get started. 
This will launch Rviz and show a 10x10 grid <br>
Then type `ros2 run agent service_test` on another terminal to display the agent marker on the grid

```
Ready to update goal pose for agent
```

To Broadcast Transforms type `ros2 service call /update_goal my_robot_interfaces/srv/UpdateGoal "goal_pose:` 
(hit tab)
```
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```
Send a Geometry Pose msg to call the service

https://github.com/N4SK4R/ROS2_tf2_Broadcaster/assets/115721424/3744acbb-10ba-40af-9f06-e63e5aaa1975




