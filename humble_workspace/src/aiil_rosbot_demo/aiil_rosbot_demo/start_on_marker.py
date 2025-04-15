import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import yaml
import os

class StartOnMarker(Node):
    def __init__(self):
        super().__init__('start_on_marker')

        # Load config
        config_file = self.declare_parameter('config_file', '').value
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as file:
                self.config = yaml.safe_load(file)
        else:
            self.get_logger().warn("Config file not found. Using default start_marker_id = 101")
            self.config = {'hazards': {'start_marker_id': 101}}

        self.start_marker_id = self.config['hazards'].get('start_marker_id', 101)
        self.marker_detected = False

        # Publisher to cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to marker ID topic (e.g., /detected_marker_id)
        self.create_subscription(Int32, '/detected_marker_id', self.marker_callback, 10)

        self.get_logger().info(f"Waiting for marker ID {self.start_marker_id} to start...")

        # Timer to publish velocity if marker was detected
        self.timer = self.create_timer(0.5, self.move_if_started)

    def marker_callback(self, msg):
        if msg.data == self.start_marker_id and not self.marker_detected:
            self.marker_detected = True
            self.get_logger().info(f"✅ Start marker {msg.data} detected. Beginning motion...")

    def move_if_started(self):
        if self.marker_detected:
            twist = Twist()
            twist.linear.x = 0.2  # Move forward
            self.cmd_pub.publish(twist)
        else:
            # Publish zero velocity to stay stopped
            self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = StartOnMarker()
    rclpy.spin(node)
    rclpy.shutdown()
