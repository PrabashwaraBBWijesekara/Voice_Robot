#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class RandomTextPublisher(Node):
    def __init__(self):
        super().__init__('random_text_publisher')
        self.publisher_ = self.create_publisher(String, '/recognizer/output', 10)
        self.timer = self.create_timer(20.0, self.publish_random_text)  # Publish every second
        self.commands = [
            "forward",
            "left",
            "right",
            "back",
            "stop",
            "halt",
            "full speed",
            "half speed",
            "move 1 m",
            "move 2 m",
            "move 3 m"
        ]
        self.get_logger().info("Random text publisher is ready to send test commands.")

    def publish_random_text(self):
        random_command = random.choice(self.commands)
        msg = String()
        msg.data = random_command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: '{random_command}'")

def main(args=None):
    rclpy.init(args=args)
    node = RandomTextPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down random text publisher node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
