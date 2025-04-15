import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterType
import yaml
import os

class HazardHandler(Node):
    def __init__(self):
        super().__init__('hazard_handler')

        # Load config file
        config_file = self.declare_parameter('config_file', '').value
        if config_file and os.path.isfile(config_file):
            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)
        else:
            self.get_logger().warn('Config file not found or not provided')
            self.config = {}

        self.start_marker_id = self.config.get('hazards', {}).get('start_marker_id', -1)
        self.subscribe_to_start = self.config.get('hazards', {}).get('enable_start_subscription', False)

        # Subscribe to markers
        self.subscription = self.create_subscription(
            String, '/detected_markers', self.marker_callback, 10)

        if self.subscribe_to_start:
            self.get_logger().info(f"Configured to react to start marker ID {self.start_marker_id}")

    def marker_callback(self, msg):
        marker_id = int(msg.data)

        if self.subscribe_to_start and marker_id == self.start_marker_id:
            self.get_logger().info("🚀 Start marker detected! Activating behavior...")
            self.start_behavior()

    def start_behavior(self):
        # Put your custom logic here
        self.get_logger().info("✅ Start behavior triggered!")

def main(args=None):
    rclpy.init(args=args)
    node = HazardHandler()
    rclpy.spin(node)
    rclpy.shutdown()
ß