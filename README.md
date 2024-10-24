### Perception_Task

A ROS2 python package that simulates a rgbd camera in a gazebo world and logs depth of a selected point on the RGB image wrt. the sensor frame.

### Setup and Run

```
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/Shubh152/perception_task.git
. /opt/ros/jazzy/setup.bash
colcon build
. install/setup.bash
ros2 launch perception_task main.launch.py
```

- Rviz and Gazebo windows will pop up and terminal will start logging.
- Play the Gazebo Sim. A window with name "RGB Image" is shown.
- To get space coordinates of the desired point from the image, click on the point in the image and press any key
- The space coordinates of the selected point wrt the reference frame are published on the /reference_frame/coord topic
- After the coordinates are published, the RGB Image appears again.
- The space coordinates of the selected point wrt the depth_camera frame are also published on the /depth_camera/coord topic

To print the reference_frame coordinates on a seperate terminal, open a new terminal

```
ros2 topic list
. /opt/ros/jazzy/setup.bash
ros2 topic echo /reference_frame/coord
```

To kill the process, press `CTRL + C` in the terminal where log appears and press any key with focus on image window
