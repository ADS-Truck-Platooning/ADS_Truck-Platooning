from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_nodes(context, num_trucks):
    nodes = []

    for i in range(int(num_trucks)):
        # ── circle_tracking_node
        nodes.append(
            Node(
                package="truck_detection",
                executable="circle_tracking_node",
                name=f"circle_tracking_{i}",
                output="screen",
                parameters=[{"truck_id": i},
                            {"stop_distance": 7.5}],
            )
        )

        # ── pointcloud2_roi_node
        nodes.append(
            Node(
                package="truck_detection",
                executable="pointcloud2_roi_node",
                name=f"pointcloud2_roi_{i}",
                output="screen",
                parameters=[{
                    "truck_id":       i,
                    "x_min":          0.0,
                    "x_max":         30.0,
                    "y_min":         -3.6,
                    "y_max":          3.6,
                    "roi_angle_deg": 30.0,
                }],
            )
        )

    return nodes

def launch_setup(context):
    num_trucks = LaunchConfiguration('NumTrucks').perform(context)
    return generate_nodes(context, num_trucks=num_trucks)

def generate_launch_description():
    declare_num_trucks = DeclareLaunchArgument(
        "NumTrucks",
        default_value="1",
        description="Number of trucks to launch",
    )

    return LaunchDescription([
        declare_num_trucks,
        OpaqueFunction(function=launch_setup),
    ])
