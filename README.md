## RGBD camera simulation:

This package spawn the iris drone with RGBD camera and publish the image in the correspondent topics. 

TODO: rosbridge which enables the exchange of messages from camera between ROS 1 and ROS 2.

1- Edit sdf files to include the camera plugin:

'''
cd /dir/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris
'''

- Delete iris.sdf file located in the directory above.

- Copy the content of backup_iris.sdf.jinja and replace in iris.sdf.jinja file located in the directory above. 
The file backup_iris.sdf.jinja can be found at: /this_repo_dir/px4_ros2_xrcedds/my_package/models/iris

2- Terminal 1: Make PX4 and roslaunch the gazebo simulation:

'''
cd /dir/PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo-classic_iris 
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 posix_sitl.launch
'''

3- Terminal 2: 

'''
MicroXRCEAgent udp4 -p 8888
'''

4- Terminal 3: to check rgbd camera topics - ROS1

'''
rostopic list
'''

- Open RVIZ to see the image camera:

'''
rviz 
'''

5- Terminal 4: to check px4 topics - ROS2

'''
source /opt/ros/foxy/setup.bash
ros2 topic list 
'''

- Run the offboard control example: 

'''
cd /this_repo_dir
source /opt/ros/foxy/setup.bash
colcon build
source install/local_setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
'''

## my_package

This ROS2 package spawn the iris drone with RGBD camera and publish the image in the correspondent topics. 

TODO: to run the PX4 in the ROS2 launch file

```
cd ~/dev/px4_ros2/ros_packages
source /opt/ros/foxy/setup.bash
colcon build
source install/local_setup.bash
ros2 launch my_package spawn_camera.launch.py
```

## px4_offboard

REFERENCE: https://github.com/Jaeyoung-Lim/px4-offboard
This is an example of code for takeoff, move and land the drone with PX4 and ROS2 via XRCE-DDS.

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
cd ~/dir/ros_packages
source /opt/ros/foxy/setup.bash
colcon build
source install/local_setup.bash
ros2 launch px4_offboard offboard_position_control.launch.py
```
