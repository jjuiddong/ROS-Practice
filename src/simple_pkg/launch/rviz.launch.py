import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

# this is the function launch  system will look for
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file = "skidbot.rviz"
    robot_file = "skidbot.urdf"
    package_name = "simple_pkg"

    pkg_path = os.path.join(get_package_share_directory(package_name))
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

    urdf_file = os.path.join(pkg_path, "urdf", robot_file)
    rviz_config = os.path.join(pkg_path, "rviz", rviz_file)
    
    # Robot State Publisher
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config])

    # create and return launch description object
    return LaunchDescription(
        [
            start_rviz_cmd,
            robot_state_publisher_node
        ]
    )
   
