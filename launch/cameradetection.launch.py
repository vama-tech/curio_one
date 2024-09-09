import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    person_obstacle = Node(
        package="curio_one",
        executable="facedetect2.py",
        
    )

    wake_on_facedetect = Node(
        package="curio_one",
        executable="wakeonfacedetect.py"
    )


    return LaunchDescription(
        [
            person_obstacle,
            wake_on_facedetect
        ]
    )