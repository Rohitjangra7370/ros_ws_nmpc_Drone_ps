#!/usr/bin/env python3
"""
nmpc_core_CASADI.py

Nonlinear Model Predictive Control (NMPC) Core Node for Fault‐Tolerant Quadrotor

Subscribes to:
  • /nmpc/imu_filtered   (sensor_msgs/Imu)
  • /nmpc/odom_filtered  (nav_msgs/Odometry)
  • /nmpc/fault_flag     (std_msgs/Bool)

Publishes:
  • /nmpc/commands       (std_msgs/Float32MultiArray)

Uses CasADi to formulate and solve an optimal control problem in real time:
  - States: position (x,y,z), velocity (vx,vy,vz)
  - Controls: motor thrusts for remaining N motors (normalized [-1,1])
  - Dynamics constraints: discrete‐time integration of point-mass in z
  - Input constraints:  u_min ≤ u ≤ u_max, failed motor thrust = 0
  - Cost: tracking a hover setpoint, minimize control effort
  - Solver: CasADi Opti with IPOPT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
import numpy as np
import casadi as ca

class NMPCCore(Node):
    def __init__(self):
        super().__init__('nmpc_core_CASADI')

        # NMPC parameters
        self.N = 10                # horizon length
        self.dt = 0.05             # timestep
        self.n_motors = 4
        self.u_min, self.u_max = -1.0, 1.0

        # CasADi Opti setup
        self.opti = ca.Opti()
        self.X = self.opti.variable(6, self.N+1)          # [x,y,z,vx,vy,vz]
        self.U = self.opti.variable(self.n_motors, self.N)  # motor thrusts

        # Parameters
        self.x0 = self.opti.parameter(6)     # initial state
        self.x_ref = self.opti.parameter(6)  # reference state
        self.failed_idx = self.opti.parameter()  # failed motor index (scalar)

        # Dynamics: double‐integrator in z only
        def f(x, u):
            thrust = ca.sum1(u)
            acc_z = thrust - 9.81
            return ca.vertcat(x[3], x[4], x[5], 0, 0, acc_z)

        # Constraints
        # initial state
        self.opti.subject_to(self.X[:,0] == self.x0)
        # dynamic rollout
        for k in range(self.N):
            xk = self.X[:,k]
            uk = self.U[:,k]
            # RK4 integration
            k1 = f(xk, uk)
            k2 = f(xk + 0.5*self.dt*k1, uk)
            k3 = f(xk + 0.5*self.dt*k2, uk)
            k4 = f(xk + self.dt*k3, uk)
            x_next = xk + (self.dt/6)*(k1 + 2*k2 + 2*k3 + k4)
            self.opti.subject_to(self.X[:,k+1] == x_next)
            # input bounds for all motors
            self.opti.subject_to(self.U[:,k] >= self.u_min)
            self.opti.subject_to(self.U[:,k] <= self.u_max)
            # enforce failed motor = 0
            self.opti.subject_to(self.U[self.failed_idx, k] == 0)

        # Cost: state tracking + control effort
        Q = ca.diag(ca.DM([10,10,10,1,1,1]))
        R = 0.1 * ca.DM.eye(self.n_motors)
        cost = 0
        for k in range(self.N):
            dx = self.X[:,k] - self.x_ref
            cost += ca.mtimes([dx.T, Q, dx]) + ca.mtimes([self.U[:,k].T, R, self.U[:,k]])
        self.opti.minimize(cost)

        # Solver
        self.opti.solver('ipopt', {'verbose': False}, {'max_iter': 50})

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/nmpc/commands', 10)
        self.create_subscription(Imu, '/nmpc/imu_filtered', self.imu_cb, 10)
        self.create_subscription(Odometry, '/nmpc/odom_filtered', self.odom_cb, 10)
        self.create_subscription(Bool, '/nmpc/fault_flag', self.fault_cb, 10)

        # internal state
        self.state = np.zeros(6)
        self.ref   = np.zeros(6)
        self.failed = 0
        self.active = False

        self.get_logger().info('NMPC Core node started.')

    def imu_cb(self, msg: Imu):
        # use vertical accel only
        self.state[5] = msg.linear_acceleration.z

    def odom_cb(self, msg: Odometry):
        # update full state
        self.state[0] = msg.pose.pose.position.x
        self.state[1] = msg.pose.pose.position.y
        self.state[2] = msg.pose.pose.position.z
        self.state[3] = msg.twist.twist.linear.x
        self.state[4] = msg.twist.twist.linear.y
        self.state[5] = msg.twist.twist.linear.z

    def fault_cb(self, msg: Bool):
        if msg.data and not self.active:
            self.active = True
            self.get_logger().info('Fault detected: activating NMPC')
        elif not msg.data and self.active:
            self.active = False
            self.get_logger().info('Fault cleared: deactivating NMPC')

        self.failed = int(self.failed)  # remains from trigger logic

        if self.active:
            # set parameters
            self.opti.set_value(self.x0, self.state)
            self.opti.set_value(self.x_ref, self.state)
            self.opti.set_value(self.failed_idx, self.failed)
            # solve
            sol = self.opti.solve()
            u_opt = sol.value(self.U)[:,0]

            # publish first control step
            cmd = Float32MultiArray()
            cmd.data = [float(u_opt[i]) for i in range(self.n_motors)]
            self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = NMPCCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()

if __name__ == '__main__':
    main()
