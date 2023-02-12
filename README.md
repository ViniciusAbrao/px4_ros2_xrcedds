This is an example of code for takeoff, move and land the drone with PX4 and ROS2 via XRCD-DDS.

1 - Simulation setup:

https://youtu.be/f5qEXDTZOug

2- Terminal 1: Open Gazebo simulation with PX4

```
cd ~/dir/PX4-Autopilot
make px4_sitl gazebo_iris
```

(Note that `make px4_sitl gazebo` no longer works).

3- Open QGroundControl to monitore the drone

4- Terminal 2: Run the micro-ROS agent:

```
cd ~/dir/ros_packages
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

5- Terminal 3: Run the example code:

```
cd ~/dir/ros_packages/src/px4-offboard/px4_offboard
```

Replace the file 'offboard_control.py' from this repository to the directory above and build:

```
source /opt/ros/foxy/setup.bash
colcon build
source install/local_setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```
