import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    '''
        Constructor to define params later to be used in launching
        individual nodes
    '''
    package_name="maze_solver"

    #Get the use_sim_time from launch arguments
    use_sim_time=LaunchConfiguration('use_sim_time')

    # Add Robot State Publisher launch file 
    mapper_params_file=os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')

    slam_toolbox_node= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')
            ]
        ), launch_arguments={'params_file':mapper_params_file}.items()
    )

    rviz=Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory(package_name), 'config', 'default.rviz')]]
        )
    
    # teleop=Node(
    #         package='teleop_twist_keyboard',
    #         executable='teleop_twist_keyboard',
    #         arguments="xterm -e",
    #         output='screen',
    # )
    


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml'),
            description='Use mapper_params_online_async file'),
       
        slam_toolbox_node,
        rviz,
        # teleop
    ])