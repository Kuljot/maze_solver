import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    package_name="ball_follower"

    use_sim_time=LaunchConfiguration('use_sim_time')
    # use_ros2_control=LaunchConfiguration('use_ros2_control')

    # Add Robot State Publisher launch file 
    xacro_file=os.path.join(get_package_share_directory(package_name),'description','top_camera.xacro')
    # robot_description_config=Command(['use_ros2_control:=',use_ros2_control,' sim_mode:=',use_sim_time,' xacro ',xacro_file])        
    robot_description_config=xacro.process_file(xacro_file)        


    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        # DeclareLaunchArgument(
        #     'use_ros2_control',
        #     default_value='true',
        #     description='Use ros2_control if true'),

        node_robot_state_publisher
    ])