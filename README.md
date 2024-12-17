"# Crowd_avoidance-w-ubuntu22-raspi-kobuki" 

Welcome to Crowd_avoidance-w-ubuntu22-raspi-kobuki!

By following this tutorial, you will be able to run a basic crowd_avoidance algorithm.
This github project was designed for specific hardware but the programs can be adjusted.

System overview

Hardware

Raspberry pi 4
Ydlidar X2L
Kobuki base
micro sd card


Software


Raspberry pi 4:
OS: Ubuntu 22.04
Python version: 3.8.10
Cuda version: 11.5
ROS version: Humble 
Turtlebot2i:

Setup

The first step is to flash ubuntu 22.04 (64bits) image to your SD Card from Raspberry Pi Imager. (Note: If rapberry py 2,3 and 4 use ubuntu server 64 bits)
Once you have flashed the image you finish installing the system. 

Step 1: Install Ubuntu 22.04 Server/Desktop
Flash Ubuntu 22.04 (64-bit) to your microSD card using Raspberry Pi Imager.
Boot up the Raspberry Pi 4.


Step 2: Update System
Ensure the system is up to date.

bash
Copy code
sudo apt update && sudo apt upgrade -y


Step 3: Install ROS2 (Humble)
Set up sources and keys:

bash
Copy code
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
Install ROS2 Humble Desktop (includes rviz2, tools, etc.):
bash
Copy code
sudo apt update && sudo apt install -y ros-humble-desktop
Set up ROS2 Environment:
Add the following to your ~/.bashrc file for persistent setup.

bash
Copy code
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

Step 4: Install Build Tools and Colcon
Install development tools and colcon for building packages.

bash
Copy code
sudo apt install -y python3-pip python3-colcon-common-extensions python3-rosdep

Initialize and update rosdep:

bash
Copy code
sudo rosdep init
rosdep update

Step 5: Install Dependencies
YDLidar SDK (required for YDLidar ROS2 driver):

bash
Copy code
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
cmake ..
make -j$(nproc)
sudo make install

Problem encountered, can't locate ydlidar_config.h when make. SOLUTION

1. Clean Build Directory
The build directory might have stale files or incorrect configurations. Start fresh.

bash
Copy code
cd ~/YDLidar-SDK
rm -rf build
mkdir build
cd build
2. Configure and Generate CMake Files
Re-run CMake with proper options:

bash
Copy code
cmake ..
If the ydlidar_config.h is still not generated, check the output of the CMake command. Look for any warnings or missing dependencies.

3. Locate the Missing ydlidar_config.h
The ydlidar_config.h file is typically generated during the cmake step or located in a specific directory.

Check if the file is missing from build:
bash
Copy code
find ~/YDLidar-SDK -name ydlidar_config.h
If it doesn’t exist, that confirms a configuration issue during CMake.

4. Manually Generate or Copy the File
If the file is not generated, some versions of YDLidar-SDK might require you to manually include it:

Locate a sample ydlidar_config.h template. It may exist under src or core directories.
bash
Copy code
cd ~/YDLidar-SDK
cp ./src/ydlidar_config.h.in ./core/common/ydlidar_config.h
Or generate the file from the .in template if available.

5. Retry Building the SDK
Run the build process again:

bash
Copy code
cd ~/YDLidar-SDK/build
cmake ..
make -j$(nproc)



YDLidar ROS2 Driver:

bash
Copy code
cd ~
mkdir -p ros2_ws/src
cd ydlidar_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git branch -humble

Build the Workspace:

bash
Copy code
cd ~/
colcon build --symlink-install
source install/setup.bash
echo "source ~/ydlidar_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

Step 6: Install Required Tools
SLAM and Navigation2:
Install Navigation2 stack and SLAM tool:

bash
Copy code
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox

People Detection Tools (optional packages for visualization):

bash
Copy code
sudo apt install -y ros-humble-openvino-node ros-humble-rviz2

Step 7: Setup Serial Port for YDLidar
Grant permissions for the Lidar’s serial port (assumes /dev/ttyUSB0):

bash
Copy code
sudo chmod 666 /dev/ttyUSB0

Step 8: Install Kobuki ROS2 Packages
Install kobuki_ros2 (or its forked driver for ROS2 Humble):

bash
Copy code
sudo apt update
sudo mkdir kobuki_ws
sudo mkdir kobuki_ws/src
cd ~/kobuki_ws/src
git clone https://github.com/kobuki-base/kobuki_ros.git

Step 9: Verify Kobuki Base Connection
Plug in the Kobuki base via USB and find the serial port:

bash
Copy code
ls /dev/ttyUSB*

Grant access to the port (assume it's /dev/ttyUSB1):

bash
Copy code
sudo chmod 666 /dev/ttyUSB1

Launch the Kobuki base driver node to check connectivity:

bash
Copy code
ros2 launch kobuki_node kobuki_node.launch.py device:=/dev/ttyUSB1

Check if topics like /odom (odometry) and /cmd_vel (velocity commands) are published:

bash
Copy code
ros2 topic list

Step 10: Launch YDLidar Driver
Run the YDLidar ROS2 Driver:

bash
Copy code
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
Verify Scan Topic:
Check the /scan topic:

bash
Copy code
ros2 topic echo /scan

Step 11: Run SLAM for Mapping
Launch the SLAM Toolbox for real-time mapping:

bash
Copy code
ros2 launch slam_toolbox online_async_launch.py

Visualize the map in RViz:

bash
Copy code
rviz2

Add:
LaserScan topic: /scan
Map topic: /map

Step 12: Run Navigation2
Launch the Navigation2 stack with your map and setup:

bash
Copy code
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False map:=/path/to/map.yaml

Step 13: Run People Detection Node
Assuming you are using a node that publishes people positions to /people_positions:

bash
Copy code
ros2 run people_detection_pkg people_detection_node


Step 14: Build and Run
Add the node to your workspace:
Create a ROS2 package if you don’t have one:

bash
Copy code
cd ~/ydlidar_ws/src
ros2 pkg create kobuki_avoidance --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs std_msgs

Place kobuki_mover.py in the kobuki_avoidance package’s scripts folder.

Make the script executable:

bash
Copy code
chmod +x ~/ydlidar_ws/src/kobuki_avoidance/scripts/kobuki_mover.py

Build the workspace:

bash
Copy code
cd ~/ydlidar_ws
colcon build
source install/setup.bash

Run the Kobuki Movement Node:

bash
Copy code
ros2 run kobuki_avoidance kobuki_mover.py

Step 15: Full Launch Workflow
Terminal 1: Start Kobuki Driver:

bash
Copy code
ros2 launch kobuki_node kobuki_node.launch.py device:=/dev/ttyUSB1

Terminal 2: Launch LiDAR driver:

bash
Copy code
ros2 launch ydlidar_ros2_driver ydlidar_launch.py

Terminal 3: Start SLAM Toolbox:

bash
Copy code
ros2 launch slam_toolbox online_async_launch.py
Terminal 4: Start People Detection Node:

bash
Copy code
ros2 run people_detection_pkg people_detection_node
Terminal 5: Run the Kobuki Movement Logic:

bash
Copy code
ros2 run kobuki_avoidance kobuki_mover.py
Testing and Debugging

RViz2 Visualization:

View /scan topic for LiDAR data.
View /cmd_vel commands.

bash
Copy code
rviz2

Check Kobuki Motion: Monitor the Kobuki’s movement using:

bash
Copy code
ros2 topic echo /cmd_vel
