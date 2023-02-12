#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleControlMode, VehicleStatus


class OffboardControl(Node):

    def __init__(self):
        print("start init")
        super().__init__('minimal_publisher')
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
        
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos)
        
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX

        timer_period = 0.02  # seconds
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10.0
        self.omega = 0.5
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.offboard_setpoint_counter_ = 0

    def timer_callback(self):
        if self.offboard_setpoint_counter_ == 50:
            # Change to Offboard mode after 50 setpoints (1s)
            self.engage_offBoard_mode()
              
            # Arm the vehicle
            self.arm()
           
        if self.offboard_setpoint_counter_ == 1650:
            # Land and cancel timer after (33s)
            self.land()
            self.timer.cancel()
            
        if self.offboard_setpoint_counter_ < 550:
            # offboard_control_mode needs to be paired with trajectory_setpoint
            print("start counter")
            self.publish_offboard_control_mode()
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_trajectory_setpoint()
            self.offboard_setpoint_counter_ += 1

        if self.offboard_setpoint_counter_ >= 550 and self.offboard_setpoint_counter_ < 1650:
            # offboard_control_mode needs to be paired with trajectory_setpoint
            print("start counter")
            self.publish_offboard_control_mode()
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publish_trajectory_setpoint_circle()
            self.offboard_setpoint_counter_ += 1

    def vehicle_status_callback(self, msg):
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state


    def arm(self):
        print("Arm command sent")
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)

    def disarm(self):
        print('Disarm command sent')
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)
    
    def land(self):
        print('Land command sent')
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.publish_vehicle_command(msg)
        
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)
        
    def engage_offBoard_mode(self):
        print('Offboard mode command sent')
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publish_vehicle_command(msg)
        
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        
        msg.position = [0.0, 0.0, -5.0]
        msg.yaw = -3.14
        
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)

        
    def publish_trajectory_setpoint_circle(self):
        msg = TrajectorySetpoint()
              
        msg.position[0] = self.radius * np.cos(self.theta)
        msg.position[1] = self.radius * np.sin(self.theta)
        msg.position[2] = -5.0
        
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        
        self.theta = self.theta + self.omega * self.dt

    def publish_vehicle_command(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    print("start main")
   
    offboard_control = OffboardControl()
    
 

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
