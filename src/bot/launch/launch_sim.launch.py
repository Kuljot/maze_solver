import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name="maze_solver"
    world_file="./src/bot/worlds/Maze"

    # Add Robot State Publisher launch file 
    rsp= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')
            ]
        ), launch_arguments={' use_sim_time':'true', ' use_ros2_control':'false', 'world':world_file}.items()
    )
    # Add Top Camera State Publisher launch file 
    csp= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory(package_name),'launch','camera_state_publisher.launch.py')
            ]
        ), launch_arguments={' use_sim_time':'true', ' use_ros2_control':'false', 'world':world_file}.items()
    )

    gazebo_params_file=os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Add Gazebo launch file from the gazebo installation
    # ros2 launch gazebo_ros gazebo.launch.py
    gazebo= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')
            ]
        ), launch_arguments={'extra_gazebo_args':'--ros-args --params-file '+gazebo_params_file}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    # ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity bot_name

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')
    spawn_camera = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'camera_description',
                                   '-entity', 'my_camera',
                                   ],
                        output='screen')

    #Custom c++ node to take the laser scan data from /laser_controller/out topic to /scan topic
    laser_remapper = Node(
        package=package_name,
        executable="laser_remapper_node",
    )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_broad"],
    # )

    return LaunchDescription([
        rsp,
        csp,
        gazebo,
        spawn_entity,
        spawn_camera,
        laser_remapper
        # diff_drive_spawner,
        # joint_broad_spawner
    ])





