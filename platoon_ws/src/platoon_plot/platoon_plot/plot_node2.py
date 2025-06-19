import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import csv

class VelocityPlotNode(Node):
    def __init__(self):
        super().__init__('velocity_plot_node')
        self.ref_vels = []
        self.ego_vels = []
        self.times = []
        self.start_time = self.get_clock().now().nanoseconds * 1e-9

        self.latest_ref_vel = 0.0
        self.latest_ego_vel = 0.0

        # CSV 파일 설정
        self.csv_filename = "velocity_log.csv"
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time_sec', 'reference_velocity', 'ego_velocity'])

        # 실시간 plot 설정
        self.fig, self.ax = plt.subplots()
        self.line2, = self.ax.plot([], [], label="ego_velocity")
        self.line1, = self.ax.plot([], [], label="reference_velocity", linestyle='--')
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Velocity (m/s)")
        self.ax.set_ylim(-2.0, 14.0)
        self.ax.grid()
        self.ax.legend()
        plt.ion()
        plt.show()

        # 구독자
        self.create_subscription(Float32, '/truck0/reference_velocity', self.ref_callback, 10)
        self.create_subscription(Float32, '/truck0/velocity', self.ego_callback, 10)

        self.get_logger().info("VelocityPlotNode running: plotting + logging in real-time.")

    def ref_callback(self, msg):
        self.latest_ref_vel = msg.data
        self.update_plot()

    def ego_callback(self, msg):
        self.latest_ego_vel = msg.data
        self.update_plot()

    def update_plot(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.start_time

        self.times.append(t)
        self.ref_vels.append(self.latest_ref_vel)
        self.ego_vels.append(self.latest_ego_vel)

        # 실시간 plot 갱신
        self.line1.set_data(self.times, self.ref_vels)
        self.line2.set_data(self.times, self.ego_vels)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.001)

        # CSV 기록
        self.csv_writer.writerow([t, self.latest_ref_vel, self.latest_ego_vel])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPlotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
