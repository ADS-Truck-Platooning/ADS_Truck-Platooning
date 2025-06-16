import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_nodes(context, *, num_trucks):
    nodes = []

    for i in range(int(num_trucks)):
        node = Node(
            package='longitudinal_control',
            executable='longitudinal_control_node',
            name=f'longitudinal_control_{i}',
            output='screen',
            parameters=[
                {'gap_kp': 0.5},
                {'gap_kd': 0.1},
                {'desired_gap': 8.0},
                {'vel_kp': 0.8},
                {'vel_ki': 2.0},
                {'k_aw': 0.25},
                {'throttle_limit': 1.0},
                {'ff_gain': 1.0},
                {'truck_id': i},
                {'desired_velocity': 5.0},
                {'velocity_decay_rate': 0.5}
            ]
        )
        nodes.append(node)
        
    return nodes

def launch_setup(context):
    num_trucks = LaunchConfiguration('NumTrucks').perform(context)
    return generate_nodes(context, num_trucks=num_trucks)

def generate_launch_description():
    declare_num_trucks = DeclareLaunchArgument(
        'NumTrucks',
        default_value='1',
        description='Number of trucks to launch'
    )

    return LaunchDescription([
        declare_num_trucks,
        OpaqueFunction(function=launch_setup)
    ])
