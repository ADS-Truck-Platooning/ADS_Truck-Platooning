#!/usr/bin/env python3

from .class_image_processor import ImageProcessor
from .cam_pixel_world_transform import convert_pixel_to_world, convert_world_to_pixel

from sensor_msgs.msg import Image

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import qos_profile_sensor_data
import argparse

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import Bool
import math

import cv2

class LaneDetection(Node):
    def __init__(self, truck_id):
        self.truck_id = truck_id
        node_name = f'truck{self.truck_id}_lane_detection'
        super().__init__(node_name)
        
        self.image_processor = ImageProcessor()

        topic_name = f'/truck{self.truck_id}/front_camera'
        self.image_sub = self.create_subscription(
            Image,
            topic_name,
            self.img_callback,
            qos_profile_sensor_data)
        self.image_sub

        # Pose Subscriber (앞차 위치)
        self.front_pose_sub = self.create_subscription(
            Pose,
            f'/truck{self.truck_id}/front_truck_pose',
            self.front_pose_callback,
            10
        )

        topic_name = f'/platoon/truck{self.truck_id}/path'
        self.path_publisher = self.create_publisher(Path, topic_name, 10)

        self.bridge = CvBridge()
        self.image = None

        self.declare_parameter("camera_on", True)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.camera_on = self.get_parameter("camera_on").get_parameter_value().bool_value
        self.camera_on_pub = self.create_publisher(Bool, f"/truck{self.truck_id}/camera_on", 10)

    def img_callback(self, msg):
        try:
            relative_points = []
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if not self.camera_on:
                return

            middle_points = self.image_processor.frame_processor(self.image, self.truck_id)

            for point in middle_points:
                relative_points.append(convert_pixel_to_world(*point))            

        except CvBridgeError as e:
            print(e)

        finally:
            self.publish_path(relative_points)

    def front_pose_callback(self, msg: Pose):
        try:
            distance_m = msg.position.x
            self.image_processor.front_distance_m = distance_m

            if self.truck_id == 1 and not self.camera_on and self.image is not None:
                try:
                    pixel_point = convert_world_to_pixel(msg.position.x, msg.position.y, Z=1.5)
                    if pixel_point is not None:
                        u, v = int(pixel_point[0]), int(pixel_point[1])
                        cv2.circle(self.image, (u, v), 5, (0, 0, 255), -1)
                        cv2.imshow(f"Camera Off Mode (Set Front Truck Position as LAP)", self.image)
                        cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().warn(f"픽셀 변환 또는 표시 실패: {e}")

        except Exception as e:
            self.get_logger().warn(f"Pose 처리 실패: {e}")

    def parameter_callback(self, params):
        for param in params:
            if param.name == "camera_on":
                self.camera_on = param.value
                self.get_logger().info(f"camera_on set to {self.camera_on}")
                self.publish_camera_on_status()
        return SetParametersResult(successful=True)

    def publish_camera_on_status(self):
        msg = Bool()
        msg.data = self.camera_on
        self.camera_on_pub.publish(msg)
    
    def publish_path(self, relative_points):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in relative_points:
            if point is None:
                continue
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = path_msg.header.stamp
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = 0.0
            path_msg.poses.append(pose_stamped)

        self.path_publisher.publish(path_msg)
    
    def yaw_to_quaternion(self, yaw):
        quaternion = Quaternion()
        quaternion.w = math.cos(yaw / 2.0)
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)

        return quaternion
            
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Lane Detection Node')
    parser.add_argument('--truck_id', type=int, help='Truck ID')
    parsed_args, _ = parser.parse_known_args()
    lane_detector = LaneDetection(truck_id=parsed_args.truck_id)
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
