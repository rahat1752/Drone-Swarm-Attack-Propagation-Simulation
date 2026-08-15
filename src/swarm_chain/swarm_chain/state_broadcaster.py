#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


class StateBroadcaster(Node):
    """Republishes PX4 local positions into swarm-level PoseStamped topics."""

    def __init__(self):
        super().__init__("state_broadcaster")

        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.state_publishers = {}
        self.subscribers = {}

        for drone_id in range(1, 6):
            self.subscribers[drone_id] = self.create_subscription(
                VehicleLocalPosition,
                f"/px4_{drone_id}/fmu/out/vehicle_local_position_v1",
                lambda msg, drone_id=drone_id: self.position_callback(msg, drone_id),
                self.px4_qos,
            )

            self.state_publishers[drone_id] = self.create_publisher(
                PoseStamped,
                f"/swarm/drone_{drone_id}_state_raw",
                10,
            )

        self.get_logger().info("State Broadcaster Started")

    def position_callback(self, msg: VehicleLocalPosition, drone_id: int):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"

        pose.pose.position.x = float(msg.x)
        pose.pose.position.y = float(msg.y)
        pose.pose.position.z = float(msg.z)

        self.state_publishers[drone_id].publish(pose)


def main():
    rclpy.init()
    node = StateBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
