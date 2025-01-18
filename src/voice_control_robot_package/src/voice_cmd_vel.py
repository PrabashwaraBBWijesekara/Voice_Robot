#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import time

class VoiceCmdVel(Node):
    def __init__(self):
        super().__init__('voice_cmd_vel')

        # Initialize speed and Twist message
        self.speed = 0.2
        self.twist_msg = Twist()

        # Publishers
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_bool = self.create_publisher(Bool, '/voice_command', 10)

        # Subscriber for voice commands
        self.create_subscription(String, '/recognizer/output', self.speech_cb, 10)

        # Log node initialization
        self.get_logger().info('Voice Command Node Initialized')

    def speech_cb(self, speech_msg):
        """Callback for processing voice commands."""
        self.get_logger().info(f"Received command: {speech_msg.data}")

        # Speed adjustment
        if "full speed" in speech_msg.data and self.speed == 0.2:
            self.speed = 0.4
        elif "half speed" in speech_msg.data and self.speed == 0.4:
            self.speed = 0.2

        # Movement commands
        if "forward" in speech_msg.data:
            self.twist_msg.linear.x = self.speed
            self.twist_msg.angular.z = 0.0
        elif "back" in speech_msg.data:
            self.twist_msg.linear.x = -self.speed
            self.twist_msg.angular.z = 0.0
        elif "left" in speech_msg.data:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = self.speed * 2.0
        elif "right" in speech_msg.data:
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = -self.speed * 2.0
        elif "move 1 m" in speech_msg.data:
            self.move_fixed_distance(5)
        elif "move 2 m" in speech_msg.data:
            self.move_fixed_distance(10)
        elif "move 2 m" in speech_msg.data:
            self.move_fixed_distance(15)
        elif "stop" in speech_msg.data or "halt" in speech_msg.data:
            self.twist_msg = Twist()

        # Publish velocity and command acknowledgment
        self.pub_vel.publish(self.twist_msg)
        self.pub_bool.publish(Bool(data=True))

    def move_fixed_distance(self, duration):
        """Move forward for a specified duration."""
        t_end = time.time() + duration
        while time.time() < t_end and rclpy.ok():
            self.twist_msg.linear.x = self.speed
            self.twist_msg.angular.z = 0.0
            self.pub_vel.publish(self.twist_msg)
            time.sleep(0.1)

    def cleanup(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Stopping the robot...')
        self.twist_msg = Twist()
        self.pub_vel.publish(self.twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdVel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
