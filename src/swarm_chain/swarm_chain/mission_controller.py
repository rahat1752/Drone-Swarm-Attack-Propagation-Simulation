#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)


class WaypointMission:
    def __init__(self, mission_file: str):
        with open(mission_file, "r") as f:
            cfg = json.load(f)

        self.altitude = float(cfg["altitude"])
        self.speed = float(cfg["speed"])
        self.waypoints = [(float(wp[0]), float(wp[1])) for wp in cfg["waypoints"]]


class MissionController(Node):


    def __init__(self):
        super().__init__("mission_controller")

        self.declare_parameter("mission_file", "")
        self.declare_parameter("leader_px4_instance", 1)
        self.declare_parameter("leader_target_system", 2)
        self.declare_parameter("timer_period_s", 0.1)
        self.declare_parameter("acceptance_radius_m", 0.5)
        self.declare_parameter("tracking_error_limit_m", 3.0)
        self.declare_parameter("takeoff_acceptance_m", 1.0)
        self.declare_parameter("arm_delay_ticks", 10)

        mission_file = str(self.get_parameter("mission_file").value)
        if not mission_file:
            raise RuntimeError("mission_file parameter is required")

        self.mission = WaypointMission(mission_file)

        self.leader_px4_instance = int(self.get_parameter("leader_px4_instance").value)
        self.leader_target_system = int(self.get_parameter("leader_target_system").value)
        self.timer_period_s = float(self.get_parameter("timer_period_s").value)
        self.acceptance_radius_m = float(self.get_parameter("acceptance_radius_m").value)
        self.tracking_error_limit_m = float(self.get_parameter("tracking_error_limit_m").value)
        self.takeoff_acceptance_m = float(self.get_parameter("takeoff_acceptance_m").value)
        self.arm_delay_ticks = int(self.get_parameter("arm_delay_ticks").value)

        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.counter = 0
        self.phase = "INIT"

        self.current_waypoint = 1 if len(self.mission.waypoints) > 1 else 0
        self.ref_x = self.mission.waypoints[0][0]
        self.ref_y = self.mission.waypoints[0][1]

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.have_position = False

        self.offboard_requested = False
        self.arm_requested = False
        self.takeoff_complete = False
        self.landed = False

        ns = f"/px4_{self.leader_px4_instance}"

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f"{ns}/fmu/in/offboard_control_mode",
            10,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f"{ns}/fmu/in/trajectory_setpoint",
            10,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            f"{ns}/fmu/in/vehicle_command",
            10,
        )
        self.phase_pub = self.create_publisher(
            String,
            "/swarm/mission_phase",
            10,
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            f"{ns}/fmu/out/vehicle_local_position_v1",
            self.position_callback,
            self.px4_qos,
        )

        self.timer = self.create_timer(self.timer_period_s, self.timer_callback)

        self.get_logger().info(
            f"Mission Controller Started | speed={self.mission.speed} m/s, altitude={self.mission.altitude} m"
        )

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def set_phase(self, phase: str):
        if self.phase != phase:
            self.phase = phase
            self.get_logger().info(f"Mission phase: {phase}")

        msg = String()
        msg.data = self.phase
        self.phase_pub.publish(msg)

    def position_callback(self, msg: VehicleLocalPosition):
        self.current_x = float(msg.x)
        self.current_y = float(msg.y)
        self.current_z = float(msg.z)
        self.have_position = True

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = self.timestamp_us()
        self.offboard_pub.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0
        msg.timestamp = self.timestamp_us()
        self.trajectory_pub.publish(msg)

    def send_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = self.leader_target_system
        msg.target_component = 1
        msg.source_system = self.leader_target_system
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.timestamp_us()
        self.command_pub.publish(msg)

    def takeoff_reached(self) -> bool:
        if not self.have_position:
            return False
        return abs(self.current_z - self.mission.altitude) <= self.takeoff_acceptance_m

    def distance_to_reference_xy(self) -> float:
        return math.sqrt((self.current_x - self.ref_x) ** 2 + (self.current_y - self.ref_y) ** 2)

    def advance_virtual_reference(self):
        if self.current_waypoint >= len(self.mission.waypoints):
            return

        if self.have_position and self.distance_to_reference_xy() > self.tracking_error_limit_m:
            return

        goal_x, goal_y = self.mission.waypoints[self.current_waypoint]
        dx = goal_x - self.ref_x
        dy = goal_y - self.ref_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.acceptance_radius_m:
            self.ref_x = goal_x
            self.ref_y = goal_y
            self.get_logger().info(f"Reached virtual waypoint {self.current_waypoint}: ({goal_x:.1f}, {goal_y:.1f})")
            self.current_waypoint += 1
            return

        step = min(self.mission.speed * self.timer_period_s, dist)
        self.ref_x += (dx / dist) * step
        self.ref_y += (dy / dist) * step

    def timer_callback(self):
        self.counter += 1

        self.publish_offboard_mode()
        self.set_phase(self.phase)

        start_x, start_y = self.mission.waypoints[0]
        takeoff_z = self.mission.altitude

        if self.counter < 50:
            self.set_phase("PREOFFBOARD")
            self.publish_position_setpoint(start_x, start_y, takeoff_z)
            return

        if self.counter == 50 and not self.offboard_requested:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.offboard_requested = True
            self.set_phase("OFFBOARD_REQUESTED")
            self.get_logger().info("Leader offboard mode requested")
            self.publish_position_setpoint(start_x, start_y, takeoff_z)
            return

        if self.counter == 50 + self.arm_delay_ticks and not self.arm_requested:
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.arm_requested = True
            self.set_phase("ARM_REQUESTED")
            self.get_logger().info("Leader arm requested")
            self.publish_position_setpoint(start_x, start_y, takeoff_z)
            return

        if not self.takeoff_complete:
            self.set_phase("TAKEOFF")
            self.publish_position_setpoint(start_x, start_y, takeoff_z)

            if self.takeoff_reached():
                self.takeoff_complete = True
                self.ref_x = start_x
                self.ref_y = start_y
                self.current_waypoint = 1 if len(self.mission.waypoints) > 1 else 0
                self.set_phase("MISSION")
                self.get_logger().info("Leader takeoff altitude reached. Starting path.")
            return

        if self.current_waypoint < len(self.mission.waypoints):
            self.set_phase("MISSION")
            self.advance_virtual_reference()
            self.publish_position_setpoint(self.ref_x, self.ref_y, self.mission.altitude)
            return

        if not self.landed:
            self.set_phase("LANDING")
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landed = True
            self.get_logger().info("Leader landing requested")
            return

        self.set_phase("LANDING")


def main():
    rclpy.init()
    node = MissionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
