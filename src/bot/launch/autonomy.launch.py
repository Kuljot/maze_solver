import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    package_name="maze_solver"

    # Add Robot State Publisher launch file 
    autonomy_node_cmd=Node(
            package=package_name,
            executable="autonomy_node",
            name="autonomy_node",
            parameters=[
                {"file": os.path.join(get_package_share_directory(package_name),'config','sim_house_locations.yaml')
            }]
        )

   
    return LaunchDescription([
        autonomy_node_cmd,
    ])





