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

### STEP 1: Creating a workspace
```
mkdir -p ~/ros2_nexus_ads_ws/src
cd ~/ros2_nexus_ads_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create docking_description --build-type ament_cmake
```

### STEP 2: Creating folders structure
docking_description/
    ├── launch/
        ├── gazebo_rviz.launch.py
    ├── urdf/
        ├── docking.xacro
    ├── worlds/
        ├── world.sdf
    ├── rviz/
        ├── docking.rviz

```
cd ~/ros2_nexus_ads_ws/src
mkdir -p docking_description/{launch,urdf,worlds,rviz}
touch docking_description/worlds/world.sdf
touch docking_description/launch/gazebo_rviz.launch.py
touch docking_description/rviz/docking.rviz
touch docking_description/urdf/docking.xacro
```

### STEP 3: Code to run robot in gazebo
- Add folder launch, urdf, worlds, rviz to CMakeLists.txt<br />
- [BUILD](#build)<br />
- Add Ground Plane, walls, objects in world.sdf
- Terminal 1: gz sim -r ~/ros2_nexus_ads_ws/src/docking_description/worlds/world.sdf
![world](image.png)
- Add robot with base_footprint, base_link, left_wheel, right_wheel, caster_wheel_back, caster_wheel_front with their joints in docking.xacro
- Terminal 1: gz sim -r ~/ros2_nexus_ads_ws/src/docking_description/worlds/world.sdf
- Terminal 2: ros2 run ros_gz_sim create   -name nexus_ads -file ~/ros2_nexus_ads_ws/src/docking_description/urdf/docking.xacro
![world_robot](image-1.png)![world_robot1](image-2.png)


# GOAL 2: Move model in Gazebo
- Add **JointStatePublisher** plugin at ending of robot.xacro
- [BUILD](#build)<br />
- Terminal 1(Start Gazebo): gz sim -r ~/ros2_nexus_amr_ws/src/robot_description/worlds/world.sdf<br />
- Terminal 2(Spawn Robot): ros2 run ros_gz_sim create   -name nexus_amr   -file ~/ros2_nexus_amr_ws/src/robot_description/urdf/robot.xacro<br/>
- Terminal 3(Create Bridge): ros2 run ros_gz_bridge parameter_bridge   /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist<br/>
- Terminal 4(Move Robot linear): ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"<br/>
- ![Demo](moving.gif)
