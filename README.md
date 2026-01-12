# ROS2_NEXUS_ADS

### Kill terminals
```
pkill -f ros2
pkill -f gazebo
pkill -f gz
pkill -f rviz
pkill -f nav2
pkill -f slam_toolbox
```

### BUILD
```
cd ~/ros2_nexus_ads_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
clear
```

# GOAL 1: Render model in Gazebo

## STEP 1: Creating a workspace
```
mkdir -p ~/ros2_nexus_ads_ws/src
cd ~/ros2_nexus_ads_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create docking_description --build-type ament_cmake
```