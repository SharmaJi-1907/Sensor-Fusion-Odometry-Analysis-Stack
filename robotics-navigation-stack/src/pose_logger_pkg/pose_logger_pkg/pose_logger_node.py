import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import math
import csv
import os
import time

def get_yaw(q):
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

class PoseLoggerNode(Node):
    def __init__(self):
        super().__init__('pose_logger_node')

        # --- PARAMETERS ---
        self.declare_parameter('wheel_odom_topic', '/mobile_base_controller/odom') 
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('log_dir', os.path.expanduser('~/pose_logs'))
        self.declare_parameter('min_dist_threshold', 0.10)
        self.declare_parameter('min_angle_threshold', 0.4)

        # Get params
        wheel_topic = self.get_parameter('wheel_odom_topic').value
        filtered_topic = self.get_parameter('filtered_odom_topic').value
        self.log_dir = self.get_parameter('log_dir').value
        self.min_dist = self.get_parameter('min_dist_threshold').value
        self.min_angle = self.get_parameter('min_angle_threshold').value

        # --- SETUP CSV ---
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        
        # Files for both topics
        self.wheel_csv_path = f"{self.log_dir}/wheel_odom_{timestamp}.csv"
        self.filtered_csv_path = f"{self.log_dir}/filtered_odom_{timestamp}.csv"

        self._init_csv(self.wheel_csv_path)
        self._init_csv(self.filtered_csv_path)

        self.get_logger().info(f"Logging wheel odom to: {self.wheel_csv_path}")
        self.get_logger().info(f"Logging filtered odom to: {self.filtered_csv_path}")

        # --- PUBLISHERS (RViz) ---
        self.wheel_path_pub = self.create_publisher(Path, 'viz/wheel_path', 10)
        self.filtered_path_pub = self.create_publisher(Path, 'viz/filtered_path', 10)

        self.wheel_path_msg = Path()
        self.filtered_path_msg = Path()

        self.last_wheel_pose = None
        self.last_filtered_pose = None

        # --- SUBSCRIBERS ---
        self.create_subscription(Odometry, wheel_topic, self.wheel_callback, 10)
        self.create_subscription(Odometry, filtered_topic, self.filtered_callback, 10)

    def _init_csv(self, filename):
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp_sec', 'x', 'y', 'z', 'yaw_rad'])

    def should_record(self, current_pose, last_pose):
        if last_pose is None:
            return True

        dx = current_pose.position.x - last_pose.position.x
        dy = current_pose.position.y - last_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)

        curr_yaw = get_yaw(current_pose.orientation)
        last_yaw = get_yaw(last_pose.orientation)
        angle_diff = abs(curr_yaw - last_yaw)
        if angle_diff > math.pi:
            angle_diff = 2*math.pi - angle_diff

        if dist >= self.min_dist or angle_diff >= self.min_angle:
            return True
        
        return False

    def append_to_csv(self, filename, pose, timestamp_sec):
        yaw = get_yaw(pose.orientation)
        with open(filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp_sec, pose.position.x, pose.position.y, pose.position.z, yaw])

    def wheel_callback(self, msg):
        current_pose = msg.pose.pose
        if self.should_record(current_pose, self.last_wheel_pose):
            self.last_wheel_pose = current_pose
            
            # RViz
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = current_pose
            self.wheel_path_msg.header = msg.header
            self.wheel_path_msg.poses.append(pose_stamped)
            self.wheel_path_pub.publish(self.wheel_path_msg)
            
            # CSV
            self.append_to_csv(self.wheel_csv_path, current_pose, msg.header.stamp.sec)

    def filtered_callback(self, msg):
        current_pose = msg.pose.pose
        if self.should_record(current_pose, self.last_filtered_pose):
            self.last_filtered_pose = current_pose
            
            # RViz
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = current_pose
            self.filtered_path_msg.header = msg.header
            self.filtered_path_msg.poses.append(pose_stamped)
            self.filtered_path_pub.publish(self.filtered_path_msg)

            # CSV
            self.append_to_csv(self.filtered_csv_path, current_pose, msg.header.stamp.sec)

def main(args=None):
    rclpy.init(args=args)
    node = PoseLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
