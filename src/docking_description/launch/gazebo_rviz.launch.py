
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
import os


def generate_launch_description():

    pkg_path = get_package_share_directory("docking_description")
    world_file = os.path.join(pkg_path, "worlds", "world.sdf")
    xacro_file = os.path.join(pkg_path, "urdf", "docking.xacro")
    rviz_file = os.path.join(pkg_path, "rviz", "docking.rviz")

    # Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen"
    )

    # Spawn robot
    spawn_robot = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'nexus_amr',
                '-file', xacro_file
            ],
            output='screen'
        )]
    )

    # bridge
    bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        output="screen"
    )

    # rsp
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file])
        }],
        output='screen'
    )
    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot,
        rviz
    ])
