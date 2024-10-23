### Perception_Task

A ROS2 python package that simulates a rgbd camera in a gazebo world and logs depth of a selected point on the RGB image wrt. the sensor frame.

### Setup and Run

```
mkdir ~/ros2_ws/
cd ~/ros2_ws/
git clone https://github.com/Shubh152/perception_task.git
. /opt/ros/jazzy/setup.bash
colcon build
. install/setup.bash
ros2 launch perception_task main.launch.py
```

- Rviz and Gazebo windows will pop up and terminal will start logging.
- Play the Gazebo Sim. A window with name "RGB Image" is shown.
- Intially depth of the centre of the RGB image is logged
- To get depth of desired coordinate, click on the coordinate in the image and press any key with focus on window
- The coordinates ofthe selected point along with the estimated depth is logged in the terminal.

To kill the process, press `CTRL + C` in the terminal where log appears and press any key with focus on image window
