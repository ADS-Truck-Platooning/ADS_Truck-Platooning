import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import math
import csv
import os

class GapPlotNode(Node):
    def __init__(self):
        super().__init__('gap_plot_node')
        self.gaps = []
        self.desired_gaps = []
        self.times = []
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.latest_desired_gap = 0.0
        self.prev_lead_x = 0.0
        self.prev_lead_y = 0.0

        # CSV 파일 준비
        self.csv_filename = "gap_log.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time_sec', 'current_gap', 'desired_gap'])

        # 실시간 plot 설정
        self.fig, self.ax = plt.subplots()
        self.line1, = self.ax.plot([], [], label="current_gap")
        self.line2, = self.ax.plot([], [], label="desired_gap", linestyle='--')
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Gap (m)")
        self.ax.set_ylim(-2.5, 30.0)
        self.ax.grid()
        self.ax.legend()
        plt.ion()
        plt.show()

        # 구독자 등록
        self.create_subscription(Pose, '/truck1/front_truck_pose', self.pose_callback, 10)
        self.create_subscription(Float32, '/desired_gap', self.desired_gap_callback, 10)

        self.get_logger().info("GapPlotNode running: plotting + logging in real-time.")

    def desired_gap_callback(self, msg):
        self.latest_desired_gap = msg.data

    def pose_callback(self, msg):
        if self.latest_desired_gap != 0.0:
            lead_x = msg.position.x
            lead_y = msg.position.y

            if lead_x == 0.0:
                lead_x = self.prev_lead_x
                lead_y = self.prev_lead_y

            gap = math.sqrt(lead_x ** 2 + lead_y ** 2)

            self.prev_lead_x = lead_x
            self.prev_lead_y = lead_y

            now = self.get_clock().now().nanoseconds * 1e-9
            t = now - self.start_time

            self.times.append(t)
            self.gaps.append(gap)
            self.desired_gaps.append(self.latest_desired_gap)

            # 실시간 plot 갱신
            self.line1.set_data(self.times, self.gaps)
            self.line2.set_data(self.times, self.desired_gaps)
            self.ax.relim()
            self.ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)

            # CSV 기록
            self.csv_writer.writerow([t, gap, self.latest_desired_gap])

    def destroy_node(self):
        # 노드 종료 시 파일 닫기
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GapPlotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
