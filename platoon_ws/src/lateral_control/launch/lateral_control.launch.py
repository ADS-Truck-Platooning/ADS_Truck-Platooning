import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_nodes(context, *, num_trucks, look_ahead_distance):
    nodes = []

    for i in range(int(num_trucks)):
        node = Node(
            package='lateral_control',
            executable='lateral_control_node',
            name=f'lateral_control_{i}',
            output='screen',
            parameters=[
                {"truck_id": i},
                {"look_ahead_distance": float(look_ahead_distance)}
            ],
            on_exit=Shutdown()
        )
        nodes.append(node)
        
    return nodes

def launch_setup(context):
    num_trucks = LaunchConfiguration('NumTrucks').perform(context)
    look_ahead_distance = LaunchConfiguration('LAD').perform(context)
    return generate_nodes(context, num_trucks=num_trucks, look_ahead_distance=look_ahead_distance)

def generate_launch_description():
    declare_num_trucks = DeclareLaunchArgument(
        'NumTrucks',
        default_value='1',
        description='Number of trucks to launch'
    )

    declare_look_ahead_distance = DeclareLaunchArgument(
        'LAD',
        default_value='20.0',
        description='Look Ahead Distance'
    )

    return LaunchDescription([
        declare_num_trucks,
        declare_look_ahead_distance,
        OpaqueFunction(function=launch_setup)
    ])
