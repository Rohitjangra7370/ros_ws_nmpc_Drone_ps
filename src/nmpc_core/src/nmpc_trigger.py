#!/usr/bin/env python3
"""
nmpc_trigger.py

Subscribes to:
  • /nmpc/motor_feedback (std_msgs/Float32MultiArray)

Publishes:
  • /nmpc/fault_flag   (std_msgs/Bool)
  • /nmpc/failed_motor (std_msgs/Int32)

Detects when any motor value falls below a failure threshold for a sustained period.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32MultiArray
from collections import deque

class NMPCTrigger(Node):
    def __init__(self):
        super().__init__('nmpc_trigger')

        # Failure threshold: normalized motor command below which we consider failure
        self.declare_parameter('failure_threshold', 0.1)
        self.failure_threshold = self.get_parameter('failure_threshold').value

        # Debounce window size (number of consecutive readings)
        self.declare_parameter('debounce_count', 5)
        self.debounce_count = self.get_parameter('debounce_count').value

        # Buffer to hold last N motor feedback arrays
        self.history = deque(maxlen=self.debounce_count)

        # Publisher for fault flag
        self.flag_pub = self.create_publisher(Bool, '/nmpc/fault_flag', 10)
        # Publisher for failed motor index
        self.index_pub = self.create_publisher(Int32, '/nmpc/failed_motor', 10)

        # Subscriber to motor feedback
        self.create_subscription(
            Float32MultiArray,
            '/nmpc/motor_feedback',
            self.motor_feedback_callback,
            10
        )

        self.get_logger().info('nmpc_trigger node started.')

    def motor_feedback_callback(self, msg: Float32MultiArray):
        """
        Collect motor feedback vectors; when one motor is low for
        debounce_count consecutive readings, trigger fault.
        """
        # Store latest feedback
        self.history.append(msg.data)

        # Only check when buffer is full
        if len(self.history) < self.debounce_count:
            return

        # Check each motor index
        for i in range(len(self.history[0])):
            # If in all buffered readings this motor is below threshold
            if all(sample[i] < self.failure_threshold for sample in self.history):
                # Publish fault flag and index once
                fault_msg = Bool(data=True)
                idx_msg = Int32(data=i)
                self.flag_pub.publish(fault_msg)
                self.index_pub.publish(idx_msg)
                self.get_logger().warn(f'Motor {i} failure detected (value < {self.failure_threshold})')
                # Clear history to avoid repeated triggers until recovery
                self.history.clear()
                return

        # If no failure detected, publish false flag
        self.flag_pub.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    node = NMPCTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
