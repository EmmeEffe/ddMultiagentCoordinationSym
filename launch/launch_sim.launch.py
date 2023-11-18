
# File from github.com/joshnewans/articubot_one
# Edited by EmmeEffe

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    num_robots = 6 # TODO PARAMETER

    package_name='multi_robots' # package name

    # Create the robots
    rsp = IncludeLaunchDescription( 
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_multi_robots.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    spawnEntityArray = [rsp, gazebo]

    # Spawn Every Robot
    for i in range(num_robots):
        spawnEntityArray.append(Node(package='gazebo_ros', executable='spawn_entity.py',
                            arguments=['-topic', 'robot'+str(i+1)+'/robot_description',
                                    '-entity', 'my_robot'+str(i+1)],
                            output='screen'))


    # Launch them all!
    return LaunchDescription(spawnEntityArray)