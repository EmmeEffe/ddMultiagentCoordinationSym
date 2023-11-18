import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():


    dist = 1 # distance between each robot (m)

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    num_robots = 6 # number of robot to spawn # TODO PARAMETER

    # Process the URDF file for the robot
    pkg_path = os.path.join(get_package_share_directory('multi_robots'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')


    # Add Rviz to packages to launch
    rviz_pkg = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
    )


    LaunchDescriptionArray = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        #rviz_pkg
    ]


    for i in range(num_robots):
        robot_description_config = xacro.process_file(xacro_file, mappings={"x_pos": str(dist*i), "namespace": "robot"+str(i+1)}).toxml()
        params_robot = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
        app_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace='robot'+str(i+1),
            parameters=[params_robot],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )
        LaunchDescriptionArray.append(app_node)


    # Launch!
    return LaunchDescription(LaunchDescriptionArray)
