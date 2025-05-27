# launch/pointcloud2_roi.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_nodes(context, *, num_trucks):
    nodes = []

    for i in range(1, int(num_trucks)):
        node = Node(
            package     = 'truck_detection',
            executable  = 'pointcloud2_roi_node',
            output='screen',
            parameters  = [{
                # ROI 파라미터를 원하는 값으로 수정!
                'x_min':         0.0,
                'x_max':          30.0,
                'y_min':         -7.5,
                'y_max':          7.5,
                'roi_angle_deg':  30.0,

                # 토픽도 필요하면
                'input_topic':  '/truck1/front_lidar',
                'output_topic': '/truck1/front_lidar/roi',
                'marker_topic': '/marker'
            }]
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
