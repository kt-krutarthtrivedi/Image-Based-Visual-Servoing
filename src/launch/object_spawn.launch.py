import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

def generate_launch_description():
    rrbot_description_path = os.path.join(get_package_share_directory('rrbot_description'))
    obj_urdf_path = os.path.join(rrbot_description_path,'urdf','object.urdf')

    spawn_object = Node(package='gazebo_ros', executable='spawn_entity.py',
                       arguments=['-file', obj_urdf_path,
                                  '-entity', 'object',
                                  '-x', '2.0'],
                       output='screen')
    nodes = [
        spawn_object
    ]

    return LaunchDescription(nodes)

