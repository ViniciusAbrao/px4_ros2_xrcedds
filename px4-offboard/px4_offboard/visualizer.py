import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from rclpy.duration import Duration

from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint

from geometry_msgs.msg import PoseStamped, Point, TransformStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from tf2_ros import TransformBroadcaster


class PX4Visualizer(Node):
    def __init__(self):
        super().__init__("visualizer")

        # QoS: PX4 (subs) = BEST_EFFORT; RViz (pubs) = RELIABLE
        qos_profile_sub_px4 = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        qos_profile_pub_rviz = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- SUBSCRIÇÕES DO PX4 (tópicos ABSOLUTOS) ---
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.vehicle_attitude_callback,
            qos_profile_sub_px4,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",  # <- _v1
            self.vehicle_local_position_callback,
            qos_profile_sub_px4,
        )
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self.trajectory_setpoint_callback,
            qos_profile_sub_px4,
        )

        # --- PUBLICADORES PARA RVIZ (namespaced pelo nó/launch) ---
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped, "px4_visualizer/vehicle_pose", qos_profile_pub_rviz
        )
        self.vehicle_vel_pub = self.create_publisher(
            Marker, "px4_visualizer/vehicle_velocity", qos_profile_pub_rviz
        )
        self.vehicle_path_pub = self.create_publisher(
            Path, "px4_visualizer/vehicle_path", qos_profile_pub_rviz
        )
        self.setpoint_path_pub = self.create_publisher(
            Path, "px4_visualizer/setpoint_path", qos_profile_pub_rviz
        )
        # marcador “grande” para facilitar visualizar
        self.body_marker_pub = self.create_publisher(
            Marker, "px4_visualizer/body_marker", qos_profile_pub_rviz
        )

        # --- ESTADO ---
        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0])   # qw, qx, qy, qz (ENU)
        self.vehicle_local_position = np.array([0.0, 0.0, 0.0])  # ENU em 'odom'
        self.vehicle_local_velocity = np.array([0.0, 0.0, 0.0])  # ENU
        self.setpoint_position = np.array([0.0, 0.0, 0.0])

        self.vehicle_path_msg = Path()
        self.setpoint_path_msg = Path()
        self.trail_size = 1000

        # tolerância de tempo: carimbar msgs ligeiramente no passado
        self.msg_tf_tolerance = 0.05  # 50 ms

        # TF broadcaster (odom -> base_link)
        self.tf_br = TransformBroadcaster(self)
        self.base_frame = "base_link"

        # Timer ~20 Hz
        self.timer = self.create_timer(0.05, self.cmdloop_callback)

    # -------------------- CALLBACKS PX4 --------------------

    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        # NED (PX4) -> ENU (RViz), quaternion (qw,qx,qy,qz)
        q_enu = 1 / np.sqrt(2) * np.array(
            [
                msg.q[0] + msg.q[3],
                msg.q[1] + msg.q[2],
                msg.q[1] - msg.q[2],
                msg.q[0] - msg.q[3],
            ]
        )
        q_enu = q_enu / np.linalg.norm(q_enu)
        self.vehicle_attitude = q_enu.astype(float)

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        # NED -> ENU para posição/velocidade
        self.vehicle_local_position[0] = float(msg.y)    # ENU.x  <- NED.y
        self.vehicle_local_position[1] = float(msg.x)    # ENU.y  <- NED.x
        self.vehicle_local_position[2] = float(-msg.z)   # ENU.z  <- -NED.z

        self.vehicle_local_velocity[0] = float(msg.vy)
        self.vehicle_local_velocity[1] = float(msg.vx)
        self.vehicle_local_velocity[2] = float(-msg.vz)

    def trajectory_setpoint_callback(self, msg: TrajectorySetpoint):
        # NED -> ENU para visualizar setpoint
        self.setpoint_position[0] = float(msg.position[1])
        self.setpoint_position[1] = float(msg.position[0])
        self.setpoint_position[2] = float(-msg.position[2])

    # -------------------- HELPERS --------------------

    def vector2PoseMsg(self, frame_id, position, attitude):
        pose_msg = PoseStamped()
        now = self.get_clock().now() - Duration(seconds=self.msg_tf_tolerance)
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.orientation.w = float(attitude[0])
        pose_msg.pose.orientation.x = float(attitude[1])
        pose_msg.pose.orientation.y = float(attitude[2])
        pose_msg.pose.orientation.z = float(attitude[3])
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        return pose_msg

    def create_arrow_marker(self, id, tail, vector):
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = "odom"
        msg.header.stamp = (self.get_clock().now() - Duration(seconds=self.msg_tf_tolerance)).to_msg()
        msg.ns = "arrow"
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0

        dt = 0.3
        tail_point = Point()
        tail_point.x = float(tail[0])
        tail_point.y = float(tail[1])
        tail_point.z = float(tail[2])

        head_point = Point()
        head_point.x = float(tail[0] + dt * vector[0])
        head_point.y = float(tail[1] + dt * vector[1])
        head_point.z = float(tail[2] + dt * vector[2])

        msg.points = [tail_point, head_point]
        return msg

    # -------------------- LOOP --------------------

    def cmdloop_callback(self):
        # 1) TF dinâmica: odom -> base_link (carimbo "now")
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.vehicle_local_position[0])
        t.transform.translation.y = float(self.vehicle_local_position[1])
        t.transform.translation.z = float(self.vehicle_local_position[2])
        t.transform.rotation.w = float(self.vehicle_attitude[0])
        t.transform.rotation.x = float(self.vehicle_attitude[1])
        t.transform.rotation.y = float(self.vehicle_attitude[2])
        t.transform.rotation.z = float(self.vehicle_attitude[3])
        self.tf_br.sendTransform(t)

        # 2) Mensagens em 'odom' (carimbo "now - tolerance")
        vehicle_pose_msg = self.vector2PoseMsg("odom", self.vehicle_local_position, self.vehicle_attitude)
        self.vehicle_pose_pub.publish(vehicle_pose_msg)

        self.vehicle_path_msg.header = vehicle_pose_msg.header
        self.vehicle_path_msg.poses.append(vehicle_pose_msg)
        if len(self.vehicle_path_msg.poses) > self.trail_size:
            del self.vehicle_path_msg.poses[0]
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

        setpoint_pose_msg = self.vector2PoseMsg("odom", self.setpoint_position, self.vehicle_attitude)
        self.setpoint_path_msg.header = setpoint_pose_msg.header
        self.setpoint_path_msg.poses.append(setpoint_pose_msg)
        if len(self.setpoint_path_msg.poses) > self.trail_size:
            del self.setpoint_path_msg.poses[0]
        self.setpoint_path_pub.publish(self.setpoint_path_msg)

        velocity_msg = self.create_arrow_marker(1, self.vehicle_local_position, self.vehicle_local_velocity)
        self.vehicle_vel_pub.publish(velocity_msg)

        # marcador “grande” do corpo pra ficar fácil de ver
        body = Marker()
        body.header.frame_id = "odom"
        body.header.stamp = (self.get_clock().now() - Duration(seconds=self.msg_tf_tolerance)).to_msg()
        body.ns = "body"
        body.id = 0
        body.type = Marker.SPHERE
        body.action = Marker.ADD
        body.pose.position.x = float(self.vehicle_local_position[0])
        body.pose.position.y = float(self.vehicle_local_position[1])
        body.pose.position.z = float(self.vehicle_local_position[2])
        body.pose.orientation.w = 1.0
        body.scale.x = 0.6
        body.scale.y = 0.6
        body.scale.z = 0.2
        body.color.r = 1.0
        body.color.g = 0.9
        body.color.b = 0.0
        body.color.a = 1.0
        self.body_marker_pub.publish(body)


def main(args=None):
    rclpy.init(args=args)
    node = PX4Visualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

