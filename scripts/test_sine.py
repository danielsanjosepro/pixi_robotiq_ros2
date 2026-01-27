#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import GripperCommand


class SineWaveGripperClient(Node):
    def __init__(self):
        super().__init__("sine_wave_gripper_client")

        self._action_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

        # ---- Tunables ----
        self.min_pos = 0.0  # fully closed (m)
        self.max_pos = 0.77  # fully open (m)
        self.max_effort = 10.0  # N
        self.frequency = 0.4  # Hz (0.1 = one full cycle every 10s)
        self.publish_rate = 10  # Hz
        # -------------------

        self.start_time = time.time()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.send_sine_goal)

    def send_sine_goal(self):
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Waiting for gripper action server...")
            return

        t = time.time() - self.start_time

        # Sine wave in [0, 1]
        s = 0.5 * (1.0 + math.sin(2.0 * math.pi * self.frequency * t))

        # Map to gripper range
        position = self.min_pos + s * (self.max_pos - self.min_pos)

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.max_effort

        self._action_client.send_goal_async(goal)

        self.get_logger().debug(f"Sent gripper goal: position={position:.3f} m")


def main(args=None):
    rclpy.init(args=args)

    node = SineWaveGripperClient()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
