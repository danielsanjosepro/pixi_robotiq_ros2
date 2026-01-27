#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


class SliderGripperClient(Node):
    def __init__(self):
        super().__init__("slider_gripper_client")
        self._action_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )
        self.max_effort = 10.0

    def send_goal(self, position):
        if not self._action_client.server_is_ready():
            self.get_logger().warn("Waiting for gripper action server...")
            return
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = self.max_effort
        self._action_client.send_goal_async(goal)
        self.get_logger().info(f"Sent position: {position:.3f} m")


def main():
    rclpy.init()
    node = SliderGripperClient()

    # ROS2 spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Matplotlib UI
    fig, ax = plt.subplots(figsize=(8, 2))
    ax.set_visible(False)
    fig.suptitle("Robotiq Gripper Control")

    slider_ax = fig.add_axes([0.2, 0.4, 0.6, 0.15])
    slider = Slider(slider_ax, "Position (m)", 0.0, 0.77, valinit=0.0)

    def on_slider_change(val):
        node.send_goal(val)

    slider.on_changed(on_slider_change)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
