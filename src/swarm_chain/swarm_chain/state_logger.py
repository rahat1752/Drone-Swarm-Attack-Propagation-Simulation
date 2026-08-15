#!/usr/bin/env python3

import csv
import json
import math
import os
from datetime import datetime
from itertools import combinations
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from px4_msgs.msg import VehicleLocalPosition

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)


class StateLogger(Node):
    def __init__(self):
        super().__init__("state_logger")

        self.declare_parameter("output_dir", "data/baseline")
        self.declare_parameter("formation_file", "")
        self.declare_parameter("sample_period_s", 0.1)
        self.declare_parameter("leader_px4_instance", 1)
        self.declare_parameter("mission_start_mode", "leader_armed")
        self.declare_parameter("fallback_start_after_s", -1.0)

        output_dir = str(self.get_parameter("output_dir").value)
        formation_file = str(self.get_parameter("formation_file").value)
        self.sample_period_s = float(self.get_parameter("sample_period_s").value)
        self.leader_px4_instance = int(self.get_parameter("leader_px4_instance").value)
        self.mission_start_mode = str(self.get_parameter("mission_start_mode").value)
        self.fallback_start_after_s = float(self.get_parameter("fallback_start_after_s").value)

        if not formation_file:
            raise RuntimeError("formation_file parameter is required")

        os.makedirs(output_dir, exist_ok=True)

        with open(formation_file, "r") as f:
            cfg = json.load(f)

        self.leader_id = int(cfg.get("leader", 1))
        self.chain = cfg["chain"]
        self.hops = self.compute_hops()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.vehicle_file = open(os.path.join(output_dir, f"vehicle_log_{timestamp}.csv"), "w", newline="")
        self.vehicle_writer = csv.writer(self.vehicle_file)
        self.vehicle_writer.writerow([
            "ros_time_ns","mission_time_ns","bucket_100ms","drone_id",
            "px4_timestamp_us","last_update_ros_time_ns","state_age_ms",
            "x","y","z","vx","vy","vz","speed_xy","speed_3d"
        ])

        self.swarm_file = open(os.path.join(output_dir, f"swarm_state_log_{timestamp}.csv"), "w", newline="")
        self.swarm_writer = csv.writer(self.swarm_file)
        self.swarm_writer.writerow([
            "ros_receive_time_ns","mission_time_ns","bucket_100ms","drone_id",
            "msg_stamp_ns","transport_age_ms","x","y","z"
        ])

        self.error_file = open(os.path.join(output_dir, f"formation_error_{timestamp}.csv"), "w", newline="")
        self.error_writer = csv.writer(self.error_file)
        self.error_writer.writerow([
            "ros_time_ns","mission_time_ns","bucket_100ms","drone_id","parent_id","hop",
            "error","error_xy","error_z","expected_x","expected_y","expected_z",
            "actual_x","actual_y","actual_z","parent_update_age_ms","child_update_age_ms",
            "parent_swarm_age_ms","child_swarm_age_ms","parent_child_stamp_skew_ms"
        ])

        self.distance_file = open(os.path.join(output_dir, f"pairwise_distance_{timestamp}.csv"), "w", newline="")
        self.distance_writer = csv.writer(self.distance_file)
        self.distance_writer.writerow([
            "ros_time_ns","mission_time_ns","bucket_100ms",
            "drone_i","drone_j","distance","distance_xy","distance_z"
        ])

        self.event_file = open(os.path.join(output_dir, f"mission_events_{timestamp}.csv"), "w", newline="")
        self.event_writer = csv.writer(self.event_file)
        self.event_writer.writerow(["ros_time_ns", "mission_time_ns", "event", "details"])

        self.latest_states: Dict[int, VehicleLocalPosition] = {}
        self.latest_state_ros_time_ns: Dict[int, int] = {}

        self.latest_swarm_states: Dict[int, PoseStamped] = {}
        self.latest_swarm_receive_time_ns: Dict[int, int] = {}
        self.latest_swarm_stamp_ns: Dict[int, int] = {}

        self.mission_active = False
        self.mission_finished = False
        self.mission_start_ns: Optional[int] = None
        self.node_start_ns = self.now_ns()

        self.vehicle_subs = []
        for drone_id in range(1, 6):
            self.vehicle_subs.append(
                self.create_subscription(
                    VehicleLocalPosition,
                    f"/px4_{drone_id}/fmu/out/vehicle_local_position_v1",
                    lambda msg, drone_id=drone_id: self.vehicle_callback(msg, drone_id),
                    self.px4_qos,
                )
            )

        self.swarm_subs = []
        for drone_id in range(1, 6):
            self.swarm_subs.append(
                self.create_subscription(
                    PoseStamped,
                    f"/swarm/drone_{drone_id}_state",
                    lambda msg, drone_id=drone_id: self.swarm_callback(msg, drone_id),
                    10,
                )
            )

        # self.status_subs = []
        # for suffix in ["vehicle_status_v2", "vehicle_status_v1", "vehicle_status"]:
        #     self.status_subs.append(
        #         self.create_subscription(
        #             VehicleStatus,
        #             f"/px4_{self.leader_px4_instance}/fmu/out/{suffix}",
        #             self.status_callback,
        #             self.px4_qos,
        #         )
        #     )

        self.phase_sub = self.create_subscription(
            String,
            "/swarm/mission_phase",
            self.phase_callback,
            10,
        )

        self.timer = self.create_timer(self.sample_period_s, self.sample_callback)

        if self.mission_start_mode == "immediate":
            self.start_logging("immediate_mode", "logger started immediately")

        self.get_logger().info(
            "State Logger Started. Main logging begins on leader ARM unless mission_start_mode:=immediate."
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def mission_time_ns(self, now_ns: int) -> int:
        if self.mission_start_ns is None:
            return 0
        return now_ns - self.mission_start_ns

    def bucket_100ms(self, mission_time_ns: int) -> int:
        return int(mission_time_ns // 100_000_000)

    @staticmethod
    def stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def compute_hops(self):
        hops = {self.leader_id: 0}
        unresolved = {int(k) for k in self.chain.keys()}
        while unresolved:
            progressed = False
            for child in list(unresolved):
                parent = int(self.chain[str(child)]["parent"])
                if parent not in hops:
                    continue
                hops[child] = hops[parent] + 1
                unresolved.remove(child)
                progressed = True
            if not progressed:
                for child in unresolved:
                    hops[child] = -1
                break
        return hops

    def write_event(self, event: str, details: str = ""):
        now = self.now_ns()
        mt = self.mission_time_ns(now)
        self.event_writer.writerow([now, mt, event, details])
        self.event_file.flush()

    def start_logging(self, event: str, details: str = ""):
        if self.mission_active or self.mission_finished:
            return
        now = self.now_ns()
        self.mission_start_ns = now
        self.mission_active = True
        self.write_event(event, details)
        self.get_logger().info(f"Mission logging started: {event}")

    def stop_logging(self, event: str, details: str = ""):
        if not self.mission_active:
            return
        self.write_event(event, details)
        self.mission_active = False
        self.mission_finished = True
        self.get_logger().info(f"Mission logging stopped: {event}")

    # def status_callback(self, msg: VehicleStatus):
    #     is_armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED

    #     if self.mission_start_mode == "leader_armed":
    #         if is_armed and not self.mission_active and not self.mission_finished:
    #             self.start_logging("leader_armed_start_logging", f"arming_state={msg.arming_state}")

    #         if (not is_armed) and self.mission_active:
    #             self.stop_logging("leader_disarmed_stop_logging", f"arming_state={msg.arming_state}")

    def phase_callback(self, msg: String):
        phase = msg.data

        if self.mission_start_mode == "leader_armed":
            if phase in ["ARM_REQUESTED", "TAKEOFF", "MISSION"] and not self.mission_active and not self.mission_finished:
                self.start_logging("mission_phase_start_logging", f"phase={phase}")

            if phase == "LANDING" and self.mission_active:
                self.write_event("mission_phase_landing", f"phase={phase}")

    def vehicle_callback(self, msg: VehicleLocalPosition, drone_id: int):
        now = self.now_ns()
        self.latest_states[drone_id] = msg
        self.latest_state_ros_time_ns[drone_id] = now

    def swarm_callback(self, msg: PoseStamped, drone_id: int):
        now = self.now_ns()
        stamp_ns = self.stamp_to_ns(msg.header.stamp)

        self.latest_swarm_states[drone_id] = msg
        self.latest_swarm_receive_time_ns[drone_id] = now
        self.latest_swarm_stamp_ns[drone_id] = stamp_ns

        if not self.mission_active or self.mission_finished:
            return

        mt = self.mission_time_ns(now)
        age_ms = (now - stamp_ns) / 1e6

        self.swarm_writer.writerow([
            now, mt, self.bucket_100ms(mt), drone_id, stamp_ns, age_ms,
            float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)
        ])

    def maybe_fallback_start(self):
        if self.mission_active or self.mission_finished:
            return
        if self.fallback_start_after_s <= 0:
            return
        elapsed_s = (self.now_ns() - self.node_start_ns) / 1e9
        if elapsed_s >= self.fallback_start_after_s:
            self.start_logging("fallback_start_logging", f"elapsed_s={elapsed_s:.3f}")

    def sample_callback(self):
        self.maybe_fallback_start()

        if not self.mission_active or self.mission_finished:
            return

        now = self.now_ns()
        mt = self.mission_time_ns(now)
        bucket = self.bucket_100ms(mt)

        for drone_id in range(1, 6):
            if drone_id not in self.latest_states:
                continue

            msg = self.latest_states[drone_id]
            last_update_ns = self.latest_state_ros_time_ns.get(drone_id, now)
            state_age_ms = (now - last_update_ns) / 1e6

            vx, vy, vz = float(msg.vx), float(msg.vy), float(msg.vz)
            speed_xy = math.sqrt(vx * vx + vy * vy)
            speed_3d = math.sqrt(vx * vx + vy * vy + vz * vz)

            self.vehicle_writer.writerow([
                now, mt, bucket, drone_id, int(getattr(msg, "timestamp", 0)),
                last_update_ns, state_age_ms,
                float(msg.x), float(msg.y), float(msg.z),
                vx, vy, vz, speed_xy, speed_3d
            ])

        for child_id_str, info in self.chain.items():
            child_id = int(child_id_str)
            parent_id = int(info["parent"])

            if child_id not in self.latest_states or parent_id not in self.latest_states:
                continue

            child = self.latest_states[child_id]
            parent = self.latest_states[parent_id]

            off_x, off_y, off_z = [float(v) for v in info["offset"]]

            expected_x = float(parent.x) + off_x
            expected_y = float(parent.y) + off_y
            expected_z = float(parent.z) + off_z

            actual_x = float(child.x)
            actual_y = float(child.y)
            actual_z = float(child.z)

            error_xy = math.sqrt((actual_x - expected_x) ** 2 + (actual_y - expected_y) ** 2)
            error_z = abs(actual_z - expected_z)
            error = math.sqrt(error_xy * error_xy + error_z * error_z)

            parent_update_age_ms = (now - self.latest_state_ros_time_ns.get(parent_id, now)) / 1e6
            child_update_age_ms = (now - self.latest_state_ros_time_ns.get(child_id, now)) / 1e6

            parent_swarm_stamp_ns = self.latest_swarm_stamp_ns.get(parent_id)
            child_swarm_stamp_ns = self.latest_swarm_stamp_ns.get(child_id)

            parent_swarm_age_ms = ""
            child_swarm_age_ms = ""
            stamp_skew_ms = ""

            if parent_swarm_stamp_ns is not None:
                parent_swarm_age_ms = (now - parent_swarm_stamp_ns) / 1e6
            if child_swarm_stamp_ns is not None:
                child_swarm_age_ms = (now - child_swarm_stamp_ns) / 1e6
            if parent_swarm_stamp_ns is not None and child_swarm_stamp_ns is not None:
                stamp_skew_ms = abs(child_swarm_stamp_ns - parent_swarm_stamp_ns) / 1e6

            self.error_writer.writerow([
                now, mt, bucket, child_id, parent_id, self.hops.get(child_id, -1),
                error, error_xy, error_z,
                expected_x, expected_y, expected_z,
                actual_x, actual_y, actual_z,
                parent_update_age_ms, child_update_age_ms,
                parent_swarm_age_ms, child_swarm_age_ms, stamp_skew_ms
            ])

        for i, j in combinations(range(1, 6), 2):
            if i not in self.latest_states or j not in self.latest_states:
                continue

            a = self.latest_states[i]
            b = self.latest_states[j]

            dx = float(a.x) - float(b.x)
            dy = float(a.y) - float(b.y)
            dz = float(a.z) - float(b.z)

            dist_xy = math.sqrt(dx * dx + dy * dy)
            dist_z = abs(dz)
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            self.distance_writer.writerow([now, mt, bucket, i, j, dist, dist_xy, dist_z])

        self.vehicle_file.flush()
        self.swarm_file.flush()
        self.error_file.flush()
        self.distance_file.flush()

    def destroy_node(self):
        for f in [self.vehicle_file, self.swarm_file, self.error_file, self.distance_file, self.event_file]:
            try:
                f.flush()
                f.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = StateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
