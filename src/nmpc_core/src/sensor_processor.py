#!/usr/bin/env python3
"""
sensor_processor.py

Subscribes to:
  • /nmpc/sensor_data   (sensor_msgs/Imu)
  • /nmpc/vehicle_state (nav_msgs/Odometry)

Applies time sync and simple filtering, then republishes on:
  • /nmpc/imu_filtered  (sensor_msgs/Imu)
  • /nmpc/odom_filtered (nav_msgs/Odometry)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber

import numpy as np

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # QoS for internal topics
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # Subscribers (message_filters for sync)
        self.imu_sub = Subscriber(self, Imu, '/nmpc/sensor_data', qos_profile=qos)
        self.odom_sub = Subscriber(self, Odometry, '/nmpc/vehicle_state', qos_profile=qos)

        # Approximate time sync: 10 queue size, 0.05s slop
        self.ts = ApproximateTimeSynchronizer(
            [self.imu_sub, self.odom_sub],
            queue_size=10,
            slop=0.05)
        self.ts.registerCallback(self.synced_callback)

        # Publishers for filtered outputs
        self.imu_pub = self.create_publisher(Imu, '/nmpc/imu_filtered', 10)
        self.odom_pub = self.create_publisher(Odometry, '/nmpc/odom_filtered', 10)

        # Simple low-pass filter state
        self.alpha = 0.7
        self.last_accel = np.zeros(3)
        self.last_gyro  = np.zeros(3)
        self.last_vel   = np.zeros(3)
        self.last_pos   = np.zeros(3)

        self.get_logger().info('Sensor Processor node started.')

    def synced_callback(self, imu_msg: Imu, odom_msg: Odometry):
        # --- IMU low-pass filter ---
        accel = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z])
        gyro = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z])

        filt_accel = self.alpha * self.last_accel + (1 - self.alpha) * accel
        filt_gyro  = self.alpha * self.last_gyro  + (1 - self.alpha) * gyro

        self.last_accel = filt_accel
        self.last_gyro  = filt_gyro

        imu_out = Imu()
        imu_out.header = imu_msg.header
        imu_out.header.frame_id = imu_msg.header.frame_id

        # copy orientation = unknown: leave zeros or from odom if desired
        imu_out.orientation_covariance = imu_msg.orientation_covariance
        imu_out.angular_velocity.x = float(filt_gyro[0])
        imu_out.angular_velocity.y = float(filt_gyro[1])
        imu_out.angular_velocity.z = float(filt_gyro[2])
        imu_out.linear_acceleration.x = float(filt_accel[0])
        imu_out.linear_acceleration.y = float(filt_accel[1])
        imu_out.linear_acceleration.z = float(filt_accel[2])
        imu_out.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        imu_out.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance

        # --- Odometry simple smoothing ---
        pos = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z])
        vel = np.array([
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z])

        filt_pos = self.alpha * self.last_pos + (1 - self.alpha) * pos
        filt_vel = self.alpha * self.last_vel + (1 - self.alpha) * vel

        self.last_pos = filt_pos
        self.last_vel = filt_vel

        odom_out = Odometry()
        odom_out.header = odom_msg.header
        odom_out.header.frame_id = odom_msg.header.frame_id
        odom_out.child_frame_id = odom_msg.child_frame_id

        odom_out.pose.pose.position.x = float(filt_pos[0])
        odom_out.pose.pose.position.y = float(filt_pos[1])
        odom_out.pose.pose.position.z = float(filt_pos[2])
        odom_out.pose.pose.orientation = odom_msg.pose.pose.orientation
        odom_out.twist.twist.linear.x = float(filt_vel[0])
        odom_out.twist.twist.linear.y = float(filt_vel[1])
        odom_out.twist.twist.linear.z = float(filt_vel[2])
        odom_out.pose.covariance = odom_msg.pose.covariance
        odom_out.twist.covariance = odom_msg.twist.covariance

        # Publish filtered messages
        self.imu_pub.publish(imu_out)
        self.odom_pub.publish(odom_out)

    def destroy_node(self):
        self.get_logger().info('Shutting down Sensor Processor node.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
