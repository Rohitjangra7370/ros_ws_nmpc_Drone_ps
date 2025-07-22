#!/usr/bin/env python3
"""
uORB Listener Node for Fault-Tolerant NMPC System
This node subscribes to essential PX4 uORB topics via micro-ROS agent
and republishes processed data to internal pipeline topics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 message types (input from micro-ROS agent)
from px4_msgs.msg import SensorCombined
from px4_msgs.msg import VehicleOdometry  
from px4_msgs.msg import ActuatorMotors

# Standard ROS2 message types (output to NMPC pipeline)
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header

import math

class UorbListenerNode(Node):
    """
    Listens to PX4 uORB topics and republishes data for NMPC pipeline
    """
    
    def __init__(self):
        super().__init__('uorb_listener_node')
        
        # QoS profile for PX4 communication (best effort for real-time data)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # === SUBSCRIBERS TO PX4 uORB TOPICS ===
        self.sensor_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_combined_callback,
            qos_profile
        )
        
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback, 
            qos_profile
        )
        
        self.actuator_sub = self.create_subscription(
            ActuatorMotors,
            '/fmu/out/actuator_motors',
            self.actuator_motors_callback,
            qos_profile
        )
        
        # === PUBLISHERS TO INTERNAL NMPC PIPELINE ===
        # Topic 1: IMU data for sensor processing
        self.imu_pub = self.create_publisher(
            Imu,
            '/nmpc/sensor_data',  # Descriptive topic name for IMU data
            10
        )
        
        # Topic 2: Vehicle state for NMPC controller  
        self.odom_pub = self.create_publisher(
            Odometry,
            '/nmpc/vehicle_state',  # Descriptive topic name for pose/velocity
            10
        )
        
        # Topic 3: Motor feedback for fault detection
        self.actuator_pub = self.create_publisher(
            Float32MultiArray,
            '/nmpc/motor_feedback',  # Descriptive topic name for motor data
            10
        )
        
        self.get_logger().info('=== uORB Listener Node Started ===')
        self.get_logger().info('Subscribed to: /fmu/out/sensor_combined')
        self.get_logger().info('Subscribed to: /fmu/out/vehicle_odometry')  
        self.get_logger().info('Subscribed to: /fmu/out/actuator_motors')
        self.get_logger().info('Publishing to: /nmpc/sensor_data, /nmpc/vehicle_state, /nmpc/motor_feedback')
        
    def sensor_combined_callback(self, msg):
        """
        Process PX4 sensor_combined (IMU) and publish as sensor_msgs/Imu
        """
        # Create standard ROS2 IMU message
        imu_msg = Imu()
        
        # Header with current timestamp
        imu_msg.header = Header()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"
        
        # Angular velocity from gyroscope (rad/s)
        imu_msg.angular_velocity.x = float(msg.gyro_rad[0])
        imu_msg.angular_velocity.y = float(msg.gyro_rad[1]) 
        imu_msg.angular_velocity.z = float(msg.gyro_rad[2])
        
        # Linear acceleration from accelerometer (m/s²)
        imu_msg.linear_acceleration.x = float(msg.accelerometer_m_s2[0])
        imu_msg.linear_acceleration.y = float(msg.accelerometer_m_s2[1])
        imu_msg.linear_acceleration.z = float(msg.accelerometer_m_s2[2])
        
        # Covariance matrices (simplified - adjust based on sensor specs)
        imu_msg.angular_velocity_covariance = [0.001] * 9
        imu_msg.linear_acceleration_covariance = [0.001] * 9  
        imu_msg.orientation_covariance = [-1.0] * 9  # No orientation in sensor_combined
        
        # Publish to NMPC sensor processing pipeline
        self.imu_pub.publish(imu_msg)
        
    def vehicle_odometry_callback(self, msg):
        """
        Process PX4 vehicle_odometry and publish as nav_msgs/Odometry
        """
        # Create standard ROS2 Odometry message
        odom_msg = Odometry()
        
        # Header  
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Position (PX4 uses NED coordinate frame)
        odom_msg.pose.pose.position.x = float(msg.position[0])  # North
        odom_msg.pose.pose.position.y = float(msg.position[1])  # East  
        odom_msg.pose.pose.position.z = float(msg.position[2])  # Down
        
        # Orientation quaternion [w, x, y, z]
        odom_msg.pose.pose.orientation.w = float(msg.q[0])
        odom_msg.pose.pose.orientation.x = float(msg.q[1])
        odom_msg.pose.pose.orientation.y = float(msg.q[2]) 
        odom_msg.pose.pose.orientation.z = float(msg.q[3])
        
        # Linear velocity  
        odom_msg.twist.twist.linear.x = float(msg.velocity[0])
        odom_msg.twist.twist.linear.y = float(msg.velocity[1])
        odom_msg.twist.twist.linear.z = float(msg.velocity[2])
        
        # Angular velocity
        odom_msg.twist.twist.angular.x = float(msg.angular_velocity[0])
        odom_msg.twist.twist.angular.y = float(msg.angular_velocity[1])
        odom_msg.twist.twist.angular.z = float(msg.angular_velocity[2])
        
        # Covariance matrices (simplified)
        odom_msg.pose.covariance = [0.001] * 36
        odom_msg.twist.covariance = [0.001] * 36
        
        # Publish to NMPC state estimation pipeline
        self.odom_pub.publish(odom_msg)
        
    def actuator_motors_callback(self, msg):
        """
        Process PX4 actuator_motors and publish as Float32MultiArray for fault detection
        """
        # Create Float32MultiArray for motor feedback data
        actuator_msg = Float32MultiArray()
        
        # Extract motor control values (normalized -1 to +1 range)
        motor_values = []
        for i in range(len(msg.control)):
            if not math.isnan(msg.control[i]):
                motor_values.append(float(msg.control[i]))
            else:
                # Handle NaN values (disabled/failed motors)
                motor_values.append(0.0)
        
        # Set the data array
        actuator_msg.data = motor_values
        
        # Publish to fault detection pipeline
        self.actuator_pub.publish(actuator_msg)
        
        # Optional: Log motor status for debugging
        if len(motor_values) >= 4:  # Quadrotor assumption
            self.get_logger().debug(
                f'Motors: M1={motor_values[0]:.3f}, M2={motor_values[1]:.3f}, '
                f'M3={motor_values[2]:.3f}, M4={motor_values[3]:.3f}'
            )

def main(args=None):
    """
    Main function to run the uORB Listener Node
    """
    # Initialize ROS2 
    rclpy.init(args=args)
    
    # Create the listener node
    uorb_listener = UorbListenerNode()
    
    try:
        # Spin the node to process callbacks
        rclpy.spin(uorb_listener)
    except KeyboardInterrupt:
        uorb_listener.get_logger().info('Shutting down uORB Listener Node...')
    finally:
        # Clean shutdown
        uorb_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
