import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro
import yaml

def get_param(file_path, param_name):
    with open(file_path, 'r') as yaml_file:
        data = yaml.safe_load(yaml_file)
        if 'ros__parameters' in data['/**'] and param_name in data['/**']['ros__parameters']:
            return data['/**']['ros__parameters'][param_name]
        else:
            print("Error: '"+param_name+"' parameter not found in the YAML file.")
            return None

def generate_launch_description():

    file_path = "src/multi_robots/config/config.yaml" # Path of config file

    num_robots = get_param(file_path, 'num_robots')
    dist = get_param(file_path, 'dist_between_robots') # distance between each robot (m)

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file for the robot
    pkg_path = os.path.join(get_package_share_directory('multi_robots'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    xacro_targets_file = os.path.join(pkg_path, 'description', 'target.xacro')


    # Add Rviz to packages to launch
    rviz_pkg = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
    )


    create_target = Node(
            package='multi_robots',
            executable='create_target_positions',
            name='create_target_positions',
            output='screen',
            parameters=[file_path]
        )

    LaunchDescriptionArray = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        create_target
    ]

    for i in range(num_robots): # Create Robots
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

    for i in range(num_robots): # Create Targets
        target_description_config = xacro.process_file(xacro_targets_file, mappings={"namespace": "target"+str(i+1)}).toxml()
        params_target = {'robot_description': target_description_config, 'use_sim_time': use_sim_time}
        app_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace='target'+str(i+1),
            parameters=[params_target],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        )
        LaunchDescriptionArray.append(app_node)


    # Launch!
    return LaunchDescription(LaunchDescriptionArray)
