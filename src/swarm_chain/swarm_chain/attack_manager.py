#!/usr/bin/env python3

import json
import random
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class AttackManager(Node):


    def __init__(self):
        super().__init__("attack_manager")

        self.declare_parameter("attack_config", "")

        attack_config = str(self.get_parameter("attack_config").value)
        if not attack_config:
            raise RuntimeError("attack_config parameter is required")

        with open(attack_config, "r") as f:
            self.cfg = json.load(f)

        self.enabled = bool(self.cfg.get("enabled", False))
        self.target_drone = int(self.cfg.get("target_drone", 1))
        self.attack_type = str(self.cfg.get("attack_type", "none")).lower()

        self.start_after_mission_s = float(
            self.cfg.get("start_after_mission_s", 0.0)
        )
        self.ramp_duration_s = max(
            float(self.cfg.get("ramp_duration_s", 1.0)),
            1e-6
        )

        self.mission_started = False
        self.mission_start_ns = None

        self.raw_buffers = {
            drone_id: deque(maxlen=5000)
            for drone_id in range(1, 6)
        }

        self.last_raw_msg = {}

        self.state_pubs = {}
        self.raw_subs = {}

        for drone_id in range(1, 6):
            self.state_pubs[drone_id] = self.create_publisher(
                PoseStamped,
                f"/swarm/drone_{drone_id}_state",
                10
            )

            self.raw_subs[drone_id] = self.create_subscription(
                PoseStamped,
                f"/swarm/drone_{drone_id}_state_raw",
                lambda msg, drone_id=drone_id:
                self.raw_callback(msg, drone_id),
                10
            )

        self.phase_sub = self.create_subscription(
            String,
            "/swarm/mission_phase",
            self.phase_callback,
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info(
            f"Attack Manager Started | enabled={self.enabled}, "
            f"type={self.attack_type}, target=D{self.target_drone}"
        )

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def phase_callback(self, msg):
        if msg.data == "MISSION" and not self.mission_started:
            self.mission_started = True
            self.mission_start_ns = self.now_ns()
            self.get_logger().info("Mission phase detected. Attack timer started.")

    def attack_time_s(self):
        if not self.mission_started or self.mission_start_ns is None:
            return -1.0

        return (self.now_ns() - self.mission_start_ns) / 1e9

    def attack_active(self):
        if not self.enabled:
            return False

        t = self.attack_time_s()
        return t >= self.start_after_mission_s

    def ramp_alpha(self):
        t = self.attack_time_s()
        if t < self.start_after_mission_s:
            return 0.0

        alpha = (t - self.start_after_mission_s) / self.ramp_duration_s
        return max(0.0, min(1.0, alpha))

    def clone_pose(self, msg):
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id

        out.pose.position.x = msg.pose.position.x
        out.pose.position.y = msg.pose.position.y
        out.pose.position.z = msg.pose.position.z

        out.pose.orientation.x = msg.pose.orientation.x
        out.pose.orientation.y = msg.pose.orientation.y
        out.pose.orientation.z = msg.pose.orientation.z
        out.pose.orientation.w = msg.pose.orientation.w

        return out

    def apply_spoof(self, msg):
        alpha = self.ramp_alpha()

        spoof_cfg = self.cfg.get("spoof", {})
        b0 = spoof_cfg.get("bias_start", [0.0, 0.0, 0.0])
        b1 = spoof_cfg.get("bias_end", [0.0, 0.0, 0.0])

        bx = float(b0[0]) + alpha * (float(b1[0]) - float(b0[0]))
        by = float(b0[1]) + alpha * (float(b1[1]) - float(b0[1]))
        bz = float(b0[2]) + alpha * (float(b1[2]) - float(b0[2]))

        out = self.clone_pose(msg)
        out.pose.position.x += bx
        out.pose.position.y += by
        out.pose.position.z += bz

        return out

    def should_drop_jamming(self):
        alpha = self.ramp_alpha()

        jam_cfg = self.cfg.get("jamming", {})
        p0 = float(jam_cfg.get("drop_probability_start", 0.0))
        p1 = float(jam_cfg.get("drop_probability_end", 1.0))

        p = p0 + alpha * (p1 - p0)
        p = max(0.0, min(1.0, p))

        return random.random() < p

    def get_delayed_msg(self, delay_ms):
        target_buffer = self.raw_buffers[self.target_drone]
        if not target_buffer:
            return None

        target_time_ns = self.now_ns() - int(delay_ms * 1e6)

        candidate = None
        for stamp_ns, msg in reversed(target_buffer):
            if stamp_ns <= target_time_ns:
                candidate = msg
                break

        if candidate is None:
            candidate = target_buffer[0][1]

        return candidate

    def get_replay_msg(self):
        target_buffer = self.raw_buffers[self.target_drone]
        if not target_buffer:
            return None

        tma_cfg = self.cfg.get("tma", {})
        replay_window_ms = float(tma_cfg.get("replay_window_ms", 2000.0))

        earliest_ns = self.now_ns() - int(replay_window_ms * 1e6)

        candidates = [
            msg for stamp_ns, msg in target_buffer
            if stamp_ns >= earliest_ns
        ]

        if not candidates:
            return target_buffer[0][1]

        return random.choice(candidates)

    def tma_delay_ms(self):
        alpha = self.ramp_alpha()
        tma_cfg = self.cfg.get("tma", {})

        d0 = float(tma_cfg.get("delay_start_ms", 0.0))
        d1 = float(tma_cfg.get("delay_end_ms", 0.0))

        return d0 + alpha * (d1 - d0)

    def tma_jitter_ms(self):
        alpha = self.ramp_alpha()
        tma_cfg = self.cfg.get("tma", {})

        j0 = float(tma_cfg.get("jitter_start_ms", 0.0))
        j1 = float(tma_cfg.get("jitter_end_ms", 0.0))

        return j0 + alpha * (j1 - j0)

    def raw_callback(self, msg, drone_id):
        now = self.now_ns()

        self.raw_buffers[drone_id].append((now, msg))
        self.last_raw_msg[drone_id] = msg

        # Non-target drones always pass through.
        if drone_id != self.target_drone:
            self.state_pubs[drone_id].publish(msg)
            return

        # If attack disabled, pass target through.
        if not self.attack_active():
            self.state_pubs[drone_id].publish(msg)
            return

        # TMA is timer-driven so delayed/replayed messages are emitted regularly.
        if self.attack_type == "tma":
            return

        if self.attack_type == "spoof":
            attacked = self.apply_spoof(msg)
            self.state_pubs[drone_id].publish(attacked)
            return

        if self.attack_type == "jamming":
            if self.should_drop_jamming():
                return
            self.state_pubs[drone_id].publish(msg)
            return

        # none or unknown
        self.state_pubs[drone_id].publish(msg)

    def timer_callback(self):
        if not self.attack_active():
            return

        if self.attack_type != "tma":
            return

        tma_cfg = self.cfg.get("tma", {})
        mode = str(tma_cfg.get("mode", "delay")).lower()

        if mode == "delay":
            msg = self.get_delayed_msg(self.tma_delay_ms())

        elif mode == "jitter":
            base_delay = self.tma_delay_ms()
            jitter = self.tma_jitter_ms()
            sampled_delay = base_delay + random.uniform(-jitter, jitter)
            sampled_delay = max(0.0, sampled_delay)
            msg = self.get_delayed_msg(sampled_delay)

        elif mode == "replay":
            msg = self.get_replay_msg()

        else:
            msg = self.last_raw_msg.get(self.target_drone, None)

        if msg is None:
            return

        self.state_pubs[self.target_drone].publish(msg)


def main():
    rclpy.init()
    node = AttackManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
