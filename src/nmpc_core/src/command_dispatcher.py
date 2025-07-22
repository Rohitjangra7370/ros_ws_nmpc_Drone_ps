#!/usr/bin/env python3
"""
command_dispatcher.py

Command Dispatcher Node for Fault-Tolerant Quadrotor Control

Subscribes to:
  • /nmpc/commands      (std_msgs/Float32MultiArray)
  • /nmpc/fault_flag    (std_msgs/Bool)  
  • /nmpc/failed_motor  (std_msgs/Int32)

Publishes:
  • /fmu/in/offboard_control_mode  (px4_msgs/OffboardControlMode)
  • /fmu/in/actuator_motors        (px4_msgs/ActuatorMotors)

Purpose:
- Remains idle during normal flight (PX4 handles control)
- When motor failure detected, switches to offboard direct actuator mode
- Publishes NMPC-computed motor commands with failed motor set to NaN
- Maintains required 2Hz+ heartbeat for offboard mode
- Reverts control back to PX4 when fault clears
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# PX4 message types
from px4_msgs.msg import OffboardControlMode, ActuatorMotors

# Standard ROS2 message types
from std_msgs.msg import Bool, Int32, Float32MultiArray

import math
import time

class CommandDispatcher(Node):
    def __init__(self):
        super().__init__('command_dispatcher')
        
        # Configuration parameters
        self.declare_parameter('heartbeat_rate_hz', 10.0)
        self.declare_parameter('offboard_timeout_ms', 200)
        
        self.heartbeat_rate = self.get_parameter('heartbeat_rate_hz').value
        self.timeout_ms = self.get_parameter('offboard_timeout_ms').value
        
        # QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # === SUBSCRIBERS ===
        # Subscribe to NMPC motor commands
        self.create_subscription(
            Float32MultiArray,
            '/nmpc/commands',
            self.motor_commands_callback,
            10
        )
        
        # Subscribe to fault detection flag
        self.create_subscription(
            Bool,
            '/nmpc/fault_flag',
            self.fault_flag_callback,
            10
        )
        
        # Subscribe to failed motor index
        self.create_subscription(
            Int32,
            '/nmpc/failed_motor',
            self.failed_motor_callback,
            10
        )
        
        # === PUBLISHERS TO PX4 ===
        # Publisher for offboard control mode (heartbeat)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        
        # Publisher for direct motor commands
        self.actuator_motors_pub = self.create_publisher(
            ActuatorMotors,
            '/fmu/in/actuator_motors',
            qos_profile
        )
        
        # === STATE VARIABLES ===
        self.fault_active = False
        self.failed_motor_index = -1
        self.latest_motor_commands = []
        self.last_command_time = time.time()
        
        # Heartbeat timer (only active during fault)
        self.heartbeat_timer = None
        
        # Command timeout timer
        self.timeout_timer = self.create_timer(
            self.timeout_ms / 1000.0,  # Convert to seconds
            self.check_command_timeout
        )
        
        self.get_logger().info('=== Command Dispatcher Node Started ===')
        self.get_logger().info(f'Heartbeat rate: {self.heartbeat_rate} Hz')
        self.get_logger().info('Waiting for motor failure detection...')
        
    def fault_flag_callback(self, msg: Bool):
        """
        Handle fault flag changes - engage/disengage offboard mode
        """
        fault_detected = msg.data
        
        if fault_detected and not self.fault_active:
            # Fault detected - engage offboard mode
            self.fault_active = True
            self.get_logger().warn('🚨 MOTOR FAILURE DETECTED - Engaging offboard direct actuator mode')
            
            # Start heartbeat timer for offboard mode
            self.start_offboard_heartbeat()
            
        elif not fault_detected and self.fault_active:
            # Fault cleared - disengage offboard mode  
            self.fault_active = False
            self.get_logger().info('✅ Fault cleared - Returning control to PX4')
            
            # Stop heartbeat and send final disable command
            self.stop_offboard_heartbeat()
    
    def failed_motor_callback(self, msg: Int32):
        """
        Store the index of the failed motor
        """
        self.failed_motor_index = msg.data
        self.get_logger().info(f'Failed motor index updated: Motor {self.failed_motor_index}')
    
    def motor_commands_callback(self, msg: Float32MultiArray):
        """
        Receive motor commands from NMPC solver
        """
        self.latest_motor_commands = list(msg.data)
        self.last_command_time = time.time()
        
        # Only forward commands during fault condition
        if self.fault_active:
            self.publish_motor_commands()
    
    def start_offboard_heartbeat(self):
        """
        Start the heartbeat timer for maintaining offboard mode
        """
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.cancel()
            
        heartbeat_period = 1.0 / self.heartbeat_rate  # Convert Hz to seconds
        self.heartbeat_timer = self.create_timer(
            heartbeat_period,
            self.publish_offboard_heartbeat
        )
        
        # Send initial offboard mode message
        self.publish_offboard_control_mode(True)
        
    def stop_offboard_heartbeat(self):
        """
        Stop heartbeat timer and disable offboard mode
        """
        if self.heartbeat_timer is not None:
            self.heartbeat_timer.cancel()
            self.heartbeat_timer = None
        
        # Send final message to disable direct actuator mode
        self.publish_offboard_control_mode(False)
    
    def publish_offboard_heartbeat(self):
        """
        Maintain offboard mode with regular heartbeat (called by timer)
        """
        if self.fault_active:
            self.publish_offboard_control_mode(True)
            # Also republish motor commands if available
            if self.latest_motor_commands:
                self.publish_motor_commands()
    
    def publish_offboard_control_mode(self, enable_direct_actuator: bool):
        """
        Publish OffboardControlMode message to engage/disengage direct actuator control
        """
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Set control mode flags
        offboard_msg.position = False
        offboard_msg.velocity = False  
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.thrust_and_torque = False
        offboard_msg.direct_actuator = enable_direct_actuator  # Key setting!
        
        self.offboard_mode_pub.publish(offboard_msg)
        
        mode_str = "ENABLED" if enable_direct_actuator else "DISABLED"
        self.get_logger().debug(f'Offboard direct actuator mode: {mode_str}')
    
    def publish_motor_commands(self):
        """
        Publish motor commands with failed motor set to NaN
        """
        if not self.latest_motor_commands:
            self.get_logger().warn('No motor commands available to publish')
            return
            
        # Create ActuatorMotors message
        actuator_msg = ActuatorMotors()
        actuator_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        actuator_msg.timestamp_sample = actuator_msg.timestamp
        
        # Initialize control array (up to 12 motors supported by PX4)
        actuator_msg.control = [0.0] * 12
        
        # Set motor commands, with failed motor = NaN
        for i in range(min(len(self.latest_motor_commands), 12)):
            if i == self.failed_motor_index:
                actuator_msg.control[i] = float('nan')  # Disarm failed motor
            else:
                # Clamp values to [-1, 1] range
                cmd = max(-1.0, min(1.0, self.latest_motor_commands[i]))
                actuator_msg.control[i] = cmd
        
        self.actuator_motors_pub.publish(actuator_msg)
        
        # Log motor commands for debugging
        active_motors = []
        for i in range(len(self.latest_motor_commands)):
            if i != self.failed_motor_index:
                active_motors.append(f'M{i}={actuator_msg.control[i]:.3f}')
        
        self.get_logger().debug(
            f'Motor commands published: {", ".join(active_motors)} | '
            f'M{self.failed_motor_index}=NaN (failed)'
        )
    
    def check_command_timeout(self):
        """
        Safety check: disable offboard mode if commands stop arriving
        """
        if self.fault_active and self.latest_motor_commands:
            time_since_last_command = time.time() - self.last_command_time
            
            if time_since_last_command > (self.timeout_ms / 1000.0):
                self.get_logger().error(
                    f'⚠️  NMPC command timeout ({time_since_last_command:.2f}s) - '
                    'Disabling offboard mode for safety'
                )
                
                # Emergency: disable offboard mode
                self.fault_active = False
                self.stop_offboard_heartbeat()
    
    def destroy_node(self):
        """
        Clean shutdown - ensure offboard mode is disabled
        """
        self.get_logger().info('Shutting down Command Dispatcher...')
        
        if self.fault_active:
            # Ensure offboard mode is disabled on shutdown
            self.publish_offboard_control_mode(False)
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    dispatcher = CommandDispatcher()
    
    try:
        rclpy.spin(dispatcher)
    except KeyboardInterrupt:
        dispatcher.get_logger().info('Keyboard interrupt - shutting down')
    finally:
        dispatcher.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
