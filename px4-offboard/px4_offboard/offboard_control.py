import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.duration import Duration

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # QoS
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50
        )

        # Parâmetros
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 0.5)
        self.declare_parameter('altitude', 5.0)
        self.radius   = float(self.get_parameter('radius').value)
        self.omega    = float(self.get_parameter('omega').value)
        self.altitude = float(self.get_parameter('altitude').value)

        # Estado PX4
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Publicadores/assinantes (TÓPICOS ABSOLUTOS do PX4)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',    self.vehicle_status_cb, qos_sub)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_cb, qos_sub)

        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.pub_traj     = self.create_publisher(TrajectorySetpoint,  '/fmu/in/trajectory_setpoint',   qos_pub)
        self.pub_cmd      = self.create_publisher(VehicleCommand,      '/fmu/in/vehicle_command',       qos_pub)

        # Timer 50 Hz
        self.dt   = 0.02
        self.theta = 0.0
        self.start_time = self.get_clock().now()
        self.offboard_cmd_sent = False
        self.arm_cmd_sent      = False

        self.timer = self.create_timer(self.dt, self.loop)

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def vehicle_status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def send_vehicle_command(self, command: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        cmd = VehicleCommand()
        cmd.timestamp = self.now_us()
        cmd.param1, cmd.param2, cmd.param3 = float(p1), float(p2), float(p3)
        cmd.param4, cmd.param5, cmd.param6, cmd.param7 = float(p4), float(p5), float(p6), float(p7)
        cmd.command = command
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.pub_cmd.publish(cmd)

    def set_offboard_mode(self):
        # MAV_CMD_DO_SET_MODE = 176; custom_mode=6 (OFFBOARD)
        self.send_vehicle_command(176, p1=1.0, p2=6.0)
        self.get_logger().info('Sent DO_SET_MODE(OFFBOARD)')

    def arm(self):
        # MAV_CMD_COMPONENT_ARM_DISARM = 400; param1=1.0
        self.send_vehicle_command(400, p1=1.0)
        self.get_logger().info('Sent ARM')

    def loop(self):
        now_us = self.now_us()

        # 1) SEMPRE publicar keep-alive de OffboardControlMode (50 Hz)
        off = OffboardControlMode()
        off.timestamp = now_us
        off.position = True
        off.velocity = False
        off.acceleration = False
        off.attitude = False
        off.body_rate = False
        self.pub_offboard.publish(off)

        # 2) SEMPRE publicar TrajectorySetpoint (handshake p/ OFFBOARD)
        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        # NED: x=N, y=E, z=Down (subir = negativo)
        sp.position[0] = float(self.radius * np.cos(self.theta))
        sp.position[1] = float(self.radius * np.sin(self.theta))
        sp.position[2] = float(-self.altitude)
        # sp.yaw = 0.0
        self.pub_traj.publish(sp)

        # 3) Após ~0.6 s de stream, pedir OFFBOARD e ARM (uma vez)
        if not self.offboard_cmd_sent and (self.get_clock().now() - self.start_time) > Duration(seconds=0.6):
            self.set_offboard_mode()
            self.offboard_cmd_sent = True

        if self.offboard_cmd_sent and not self.arm_cmd_sent and (self.get_clock().now() - self.start_time) > Duration(seconds=0.8):
            self.arm()
            self.arm_cmd_sent = True

        # 4) Atualiza trajetória continuamente
        # (independente do estado; PX4 só obedecerá quando estiver OFFBOARD + ARMED)
        self.theta += self.omega * self.dt


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

