# Launch the control with the proportional control node 

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

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

    package_name='multi_robots' # package name

    file_path = "src/multi_robots/config/config.yaml"

    num_robots = get_param(file_path, 'num_robots')


    # Create the robots
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_multi_robots.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]), launch_arguments={'use_sim_time': 'true'}.items()
             )

    spawnEntityArray = [rsp, gazebo]

    # Append the create target positions node
    spawnEntityArray.append(Node(
            package=package_name,
            executable='create_target_positions',
            parameters=[file_path]))

    # Spawn Every Robot
    for i in range(num_robots):
        spawnEntityArray.append(Node(package='gazebo_ros', executable='spawn_entity.py', # Spawn robots
                            arguments=['-topic', 'robot'+str(i+1)+'/robot_description',
                                    '-entity', 'my_robot'+str(i+1)],
                            output='screen'))

        spawnEntityArray.append(Node(package='gazebo_ros', executable='spawn_entity.py', # Spawn Targets
                            arguments=['-topic', 'target'+str(i+1)+'/robot_description',
                                    '-entity', 'target'+str(i+1)],
                            output='screen'))


        # Append the com to pt node
        '''spawnEntityArray.append(Node(
            package=package_name,
            executable='com_to_pt_odom',
            namespace='robot'+str(i+1),
            parameters=[file_path],
            remappings=[('/odom', 'odom'), ('/newpt_coordinates', 'newpt_coordinates')]
        ))'''

        # Append the accel to cmd vel node
        '''spawnEntityArray.append(Node(
            package=package_name,
            executable='accel_to_cmd_vel',
            namespace='robot'+str(i+1),
            parameters=[file_path],
            remappings=[('/cmd_acc', 'cmd_acc'), ('/cmd_vel', 'cmd_vel'), ('/newpt_coordinates', 'newpt_coordinates'), ('/theta_orientation', 'theta_orientation')]
        ))'''

        spawnEntityArray.append(Node(
            package=package_name,
            executable='point_mass_dynamics',
            namespace='robot'+str(i+1),
            parameters=[file_path],
            remappings=[('/cmd_acc', 'cmd_acc'), ('/point_nav_state', 'point_nav_state')]
        ))

        spawnEntityArray.append(Node(
            package=package_name,
            executable='control_system_proportional',
            namespace='robot'+str(i+1),
            parameters=[file_path],
            remappings=[('/cmd_acc', 'cmd_acc'), ('/point_nav_state', 'point_nav_state'), ('/target_pos', 'target_pos')]
        ))

        spawnEntityArray.append(Node(
            package=package_name,
            executable='tracking_control',
            namespace='robot'+str(i+1),
            parameters=[file_path],
            remappings=[('/cmd_vel', 'cmd_vel'), ('/point_nav_state', 'point_nav_state'), ('/odom', 'odom')]
        ))

    # Launch them all!
    return LaunchDescription(spawnEntityArray)