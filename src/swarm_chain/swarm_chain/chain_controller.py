#!/usr/bin/env python3

import json
import math
from typing import Dict, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)


class ChainController(Node):
    """
    Formation follower controller.

    Followers land only when /swarm/mission_phase == LANDING.
    This avoids false landing detection from leader altitude noise or failed takeoff.
    """

    def __init__(self):
        super().__init__("chain_controller")

        self.declare_parameter("formation_file", "")
        self.declare_parameter("mission_file", "")
        self.declare_parameter("timer_period_s", 0.1)
        self.declare_parameter("follower_speed_mps", 2.0)
        self.declare_parameter("initial_hold_s", 5.0)

        formation_file = str(self.get_parameter("formation_file").value)
        mission_file = str(self.get_parameter("mission_file").value)
        if not formation_file:
            raise RuntimeError("formation_file parameter is required")

        with open(formation_file, "r") as f:
            cfg = json.load(f)

        self.leader_id = int(cfg.get("leader", 1))
        self.chain = cfg["chain"]

        self.timer_period_s = float(self.get_parameter("timer_period_s").value)
        self.follower_speed_mps = float(self.get_parameter("follower_speed_mps").value)
        self.initial_hold_s = float(self.get_parameter("initial_hold_s").value)

        self.altitude = -15.0
        if mission_file:
            with open(mission_file, "r") as f:
                mission_cfg = json.load(f)
            self.altitude = float(mission_cfg.get("altitude", self.altitude))

        self.counter = 0
        self.mission_phase = "INIT"

        self.all_states: Dict[int, PoseStamped] = {}
        self.commanded_targets: Dict[int, Tuple[float, float, float]] = {}

        self.armed: Dict[int, bool] = {}
        self.disarmed: Dict[int, bool] = {}
        self.landing_started = False

        self.offboard_pubs = {}
        self.trajectory_pubs = {}
        self.command_pubs = {}

        self.follower_ids = sorted([int(k) for k in self.chain.keys()])

        for drone_id in self.follower_ids:
            self.offboard_pubs[drone_id] = self.create_publisher(
                OffboardControlMode,
                f"/px4_{drone_id}/fmu/in/offboard_control_mode",
                10,
            )
            self.trajectory_pubs[drone_id] = self.create_publisher(
                TrajectorySetpoint,
                f"/px4_{drone_id}/fmu/in/trajectory_setpoint",
                10,
            )
            self.command_pubs[drone_id] = self.create_publisher(
                VehicleCommand,
                f"/px4_{drone_id}/fmu/in/vehicle_command",
                10,
            )
            self.armed[drone_id] = False
            self.disarmed[drone_id] = False

        self.subscribers = []
        for drone_id in range(1, 6):
            sub = self.create_subscription(
                PoseStamped,
                f"/swarm/drone_{drone_id}_state",
                lambda msg, drone_id=drone_id: self.state_callback(msg, drone_id),
                10,
            )
            self.subscribers.append(sub)

        self.phase_sub = self.create_subscription(
            String,
            "/swarm/mission_phase",
            self.phase_callback,
            10,
        )

        self.initial_global_targets = self.compute_initial_global_targets()
        for drone_id in self.follower_ids:
            self.commanded_targets[drone_id] = self.initial_global_targets[drone_id]

        self.timer = self.create_timer(self.timer_period_s, self.timer_callback)

        self.get_logger().info(
            f"Chain Controller Started | followers={self.follower_ids}, follower_speed={self.follower_speed_mps} m/s"
        )

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def compute_initial_global_targets(self) -> Dict[int, Tuple[float, float, float]]:
        targets = {self.leader_id: (0.0, 0.0, self.altitude)}
        unresolved = {int(k) for k in self.chain.keys()}

        while unresolved:
            progressed = False
            for child in list(unresolved):
                info = self.chain[str(child)]
                parent = int(info["parent"])
                if parent not in targets:
                    continue

                px, py, pz = targets[parent]
                dx, dy, dz = [float(v) for v in info["offset"]]
                targets[child] = (px + dx, py + dy, pz + dz)
                unresolved.remove(child)
                progressed = True

            if not progressed:
                raise RuntimeError("Formation graph has unresolved parent references or a cycle.")

        return targets

    def state_callback(self, msg: PoseStamped, drone_id: int):
        self.all_states[drone_id] = msg

    def phase_callback(self, msg: String):
        self.mission_phase = msg.data

    def publish_offboard_mode(self, drone_id: int):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = self.timestamp_us()
        self.offboard_pubs[drone_id].publish(msg)

    def publish_target(self, drone_id: int, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0
        msg.timestamp = self.timestamp_us()
        self.trajectory_pubs[drone_id].publish(msg)

    def send_command(self, drone_id: int, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.target_system = drone_id + 1
        msg.target_component = 1
        msg.source_system = drone_id + 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp_us()
        self.command_pubs[drone_id].publish(msg)

    def move_toward(self, current, goal):
        cx, cy, cz = current
        gx, gy, gz = goal
        dx = gx - cx
        dy = gy - cy
        dz = gz - cz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        max_step = self.follower_speed_mps * self.timer_period_s
        if dist <= max_step or dist < 1e-6:
            return goal

        scale = max_step / dist
        return (cx + dx * scale, cy + dy * scale, cz + dz * scale)

    def land_drone(self, drone_id: int):
        self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info(f"D{drone_id} landing requested")

    def disarm_drone(self, drone_id: int):
        self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.disarmed[drone_id] = True
        self.get_logger().info(f"D{drone_id} disarm requested")

    def timer_callback(self):
        self.counter += 1

        for drone_id in self.follower_ids:
            self.publish_offboard_mode(drone_id)

        initial_hold_ticks = int(self.initial_hold_s / self.timer_period_s)

        if self.counter < initial_hold_ticks:
            for drone_id in self.follower_ids:
                self.publish_target(drone_id, *self.initial_global_targets[drone_id])
            return

        if self.counter == initial_hold_ticks:
            for drone_id in self.follower_ids:
                self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.get_logger().info("Followers offboard mode requested")
            return

        if self.counter == initial_hold_ticks + 10:
            for drone_id in self.follower_ids:
                self.send_command(drone_id, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.armed[drone_id] = True
            self.get_logger().info("Followers arm requested")
            return

        if self.mission_phase == "LANDING":
            if not self.landing_started:
                self.landing_started = True
                self.get_logger().info("Mission LANDING phase received. Followers landing.")
                for drone_id in self.follower_ids:
                    self.land_drone(drone_id)
                return

            for drone_id in self.follower_ids:
                if drone_id not in self.all_states:
                    continue
                z = self.all_states[drone_id].pose.position.z
                if z > -0.2 and not self.disarmed[drone_id]:
                    self.disarm_drone(drone_id)
            return

        for child_id_str, info in self.chain.items():
            child_id = int(child_id_str)
            parent_id = int(info["parent"])

            if parent_id not in self.all_states:
                continue

            parent = self.all_states[parent_id]
            dx, dy, dz = [float(v) for v in info["offset"]]

            desired = (
                parent.pose.position.x + dx,
                parent.pose.position.y + dy,
                parent.pose.position.z + dz,
            )

            current_cmd = self.commanded_targets[child_id]
            next_cmd = self.move_toward(current_cmd, desired)
            self.commanded_targets[child_id] = next_cmd

            self.publish_target(child_id, *next_cmd)


def main():
    rclpy.init()
    node = ChainController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
