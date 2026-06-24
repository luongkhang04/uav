#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger


HELP = """
Keyboard UAV control: /uav/cmd_vel_body, frame=base_link, ROS FLU

Movement:
  w/s : forward / backward
  a/d : left / right
  r/f : up / down
  j/l : yaw left / yaw right
  space : stop

PX4 lifecycle:
  o : Offboard + Arm
  p : Land
  k : Disarm

Speed:
  + : increase speed
  - : decrease speed

Other:
  h : print help
  x : exit

Safety:
  Use k/disarm only after landing in simulation.
"""


class KeyboardCmdVel(Node):
    def __init__(self):
        super().__init__("keyboard_cmd_vel")

        self.pub = self.create_publisher(TwistStamped, "/uav/cmd_vel_body", 10)

        self.offboard_arm_client = self.create_client(Trigger, "/uav/offboard_arm")
        self.land_client = self.create_client(Trigger, "/uav/land")
        self.disarm_client = self.create_client(Trigger, "/uav/disarm")

        self.speed_xy = 1.0
        self.speed_z = 0.5
        self.yaw_rate = 0.6

        self.cmd = TwistStamped()
        self.cmd.header.frame_id = "base_link"

        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20 Hz

        print(HELP)

    def publish_cmd(self):
        self.cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cmd)

    def stop(self):
        self.cmd = TwistStamped()
        self.cmd.header.frame_id = "base_link"

    def call_trigger_service(self, client, name):
        self.stop()

        if not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f"Service {name} is not available.")
            return

        future = client.call_async(Trigger.Request())

        def done_callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info(f"{name}: {response.message}")
                else:
                    self.get_logger().warn(f"{name} failed: {response.message}")
            except Exception as exc:
                self.get_logger().error(f"{name} call failed: {exc}")

        future.add_done_callback(done_callback)

    def handle_key(self, key):
        # Lifecycle keys.
        if key == "o":
            self.call_trigger_service(self.offboard_arm_client, "/uav/offboard_arm")
            return

        if key == "p":
            self.call_trigger_service(self.land_client, "/uav/land")
            return

        if key == "k":
            self.call_trigger_service(self.disarm_client, "/uav/disarm")
            return

        if key == "h":
            print(HELP)
            return

        # Movement keys.
        self.stop()

        if key == "w":
            self.cmd.twist.linear.x = self.speed_xy
        elif key == "s":
            self.cmd.twist.linear.x = -self.speed_xy
        elif key == "a":
            self.cmd.twist.linear.y = self.speed_xy
        elif key == "d":
            self.cmd.twist.linear.y = -self.speed_xy
        elif key == "r":
            self.cmd.twist.linear.z = self.speed_z
        elif key == "f":
            self.cmd.twist.linear.z = -self.speed_z
        elif key == "j":
            self.cmd.twist.angular.z = self.yaw_rate
        elif key == "l":
            self.cmd.twist.angular.z = -self.yaw_rate
        elif key == " ":
            self.stop()
        elif key == "+":
            self.speed_xy = min(self.speed_xy + 0.2, 5.0)
            self.speed_z = min(self.speed_z + 0.1, 2.0)
            self.get_logger().info(
                f"speed_xy={self.speed_xy:.1f}, speed_z={self.speed_z:.1f}"
            )
        elif key == "-":
            self.speed_xy = max(self.speed_xy - 0.2, 0.2)
            self.speed_z = max(self.speed_z - 0.1, 0.1)
            self.get_logger().info(
                f"speed_xy={self.speed_xy:.1f}, speed_z={self.speed_z:.1f}"
            )


def main():
    rclpy.init()
    node = KeyboardCmdVel()

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.05)[0]:
                key = sys.stdin.read(1)
                if key == "x":
                    break
                node.handle_key(key)

            rclpy.spin_once(node, timeout_sec=0.0)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.stop()
        node.publish_cmd()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
