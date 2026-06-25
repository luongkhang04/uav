#!/usr/bin/env python3

import math
import os
import select
import struct
import sys
import termios
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


HELP = """
Keyboard UAV control: /uav/cmd_vel_body, frame=base_link, ROS FLU

Movement:
  hold w/s : forward / backward
  hold a/d : left / right
  hold r/f : up / down
  hold q/e : yaw left / yaw right
  release movement keys to stop that axis

PX4 lifecycle:
  1 : Offboard + Arm
  2 : Land
  3 : Disarm

Speed:
  Up arrow    : increase speed
  Down arrow  : decrease speed
  Right arrow : increase yaw rate
  Left arrow  : decrease yaw rate

Other:
  4 : print help
  x : exit

Safety:
  Use 3/disarm only after landing in simulation.
"""


EV_KEY = 0x01
KEY_RELEASE = 0
KEY_PRESS = 1
INPUT_EVENT_FORMAT = 'llHHI'
INPUT_EVENT_SIZE = struct.calcsize(INPUT_EVENT_FORMAT)

KEY_CODES = {
    2: '1',
    3: '2',
    4: '3',
    5: '4',
    16: 'q',
    17: 'w',
    18: 'e',
    19: 'r',
    30: 'a',
    31: 's',
    32: 'd',
    33: 'f',
    45: 'x',
    103: 'up',
    105: 'left',
    106: 'right',
    108: 'down',
}

MOVEMENT_KEYS = {'w', 's', 'a', 'd', 'r', 'f', 'q', 'e'}
PX4_MAX_XY_SPEED = 12.0
PX4_MAX_Z_SPEED_UP = 3.0
PX4_MAX_Z_SPEED_DOWN = 1.5
MAX_YAW_RATE = 1.0
MIN_YAW_RATE = 0.1


class LinuxKeyboard:
    def __init__(self):
        self.files = []
        self.fd_to_path = {}

        for path in self.find_keyboard_event_paths():
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except PermissionError:
                continue
            except OSError:
                continue

            self.files.append(fd)
            self.fd_to_path[fd] = path

        if not self.files:
            raise RuntimeError(
                'No readable keyboard event device found. '
                'Run with permission to read /dev/input/event* '
                'or add your user to the input group.'
            )

    def close(self):
        for fd in self.files:
            os.close(fd)

        self.files = []
        self.fd_to_path = {}

    def read_events(self, timeout_sec):
        readable, _, _ = select.select(self.files, [], [], timeout_sec)

        for fd in readable:
            try:
                data = os.read(fd, INPUT_EVENT_SIZE * 64)
            except BlockingIOError:
                continue
            except OSError:
                continue

            event_count = len(data) // INPUT_EVENT_SIZE
            for index in range(event_count):
                offset = index * INPUT_EVENT_SIZE
                event = data[offset:offset + INPUT_EVENT_SIZE]
                _, _, event_type, code, value = struct.unpack(
                    INPUT_EVENT_FORMAT, event
                )

                key = KEY_CODES.get(code)
                if event_type == EV_KEY and key is not None:
                    yield key, value

    @staticmethod
    def find_keyboard_event_paths():
        paths = []

        try:
            with open(
                '/proc/bus/input/devices', 'r', encoding='utf-8'
            ) as devices:
                blocks = devices.read().split('\n\n')
        except OSError:
            return paths

        for block in blocks:
            handlers_line = next(
                (line for line in block.splitlines() if line.startswith('H: ')),
                '',
            )

            if 'kbd' not in handlers_line:
                continue

            for handler in handlers_line.split():
                if handler.startswith('event'):
                    paths.append(f'/dev/input/{handler}')

        return paths


class TerminalMode:
    def __init__(self):
        self.fd = None
        self.old_settings = None

        if not sys.stdin.isatty():
            return

        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        new_settings = termios.tcgetattr(self.fd)
        new_settings[3] &= ~(termios.ECHO | termios.ICANON)
        new_settings[6][termios.VMIN] = 0
        new_settings[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new_settings)

    def restore(self):
        if self.fd is not None and self.old_settings is not None:
            termios.tcflush(self.fd, termios.TCIFLUSH)
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            self.fd = None
            self.old_settings = None


class TerminalKeyboard:
    def __init__(self, release_timeout_sec=0.6):
        self.fd = sys.stdin.fileno()
        self.release_timeout_sec = release_timeout_sec
        self.active_key_times = {}
        self.escape_buffer = b''

    def close(self):
        self.active_key_times.clear()

    def read_events(self, timeout_sec):
        events = []
        readable, _, _ = select.select([self.fd], [], [], timeout_sec)

        if readable:
            try:
                data = os.read(self.fd, 64)
            except BlockingIOError:
                data = b''
            except OSError:
                data = b''

            events.extend(self.parse_bytes(data))

        now = time.monotonic()
        expired_keys = [
            key for key, seen_at in self.active_key_times.items()
            if now - seen_at > self.release_timeout_sec
        ]

        for key in expired_keys:
            del self.active_key_times[key]
            events.append((key, KEY_RELEASE))

        return events

    def parse_bytes(self, data):
        events = []
        data = self.escape_buffer + data
        self.escape_buffer = b''
        index = 0

        while index < len(data):
            byte = data[index:index + 1]

            if byte == b'\x1b':
                if len(data) - index < 3:
                    self.escape_buffer = data[index:]
                    break

                sequence = data[index:index + 3]
                if sequence in {b'\x1b[A', b'\x1bOA'}:
                    events.append(('up', KEY_PRESS))
                    index += 3
                    continue
                if sequence in {b'\x1b[B', b'\x1bOB'}:
                    events.append(('down', KEY_PRESS))
                    index += 3
                    continue
                if sequence in {b'\x1b[C', b'\x1bOC'}:
                    events.append(('right', KEY_PRESS))
                    index += 3
                    continue
                if sequence in {b'\x1b[D', b'\x1bOD'}:
                    events.append(('left', KEY_PRESS))
                    index += 3
                    continue

                index += 1
                continue

            try:
                key = byte.decode('utf-8').lower()
            except UnicodeDecodeError:
                index += 1
                continue

            if key in MOVEMENT_KEYS:
                self.active_key_times[key] = time.monotonic()
                events.append((key, KEY_PRESS))
            elif key in {'1', '2', '3', '4', 'x'}:
                events.append((key, KEY_PRESS))

            index += 1

        return events


def is_ssh_session():
    return bool(os.environ.get('SSH_CONNECTION') or os.environ.get('SSH_TTY'))


class KeyboardCmdVel(Node):
    def __init__(self):
        super().__init__('keyboard_cmd_vel')

        self.pub = self.create_publisher(TwistStamped, '/uav/cmd_vel_body', 10)

        self.offboard_arm_client = self.create_client(
            Trigger, '/uav/offboard_arm'
        )
        self.land_client = self.create_client(Trigger, '/uav/land')
        self.disarm_client = self.create_client(Trigger, '/uav/disarm')

        self.speed_xy = 8.0
        self.speed_z_up = 2.0
        self.speed_z_down = 1.0
        self.yaw_rate = 0.5

        self.cmd = TwistStamped()
        self.cmd.header.frame_id = 'base_link'
        self.active_keys = set()

        self.timer = self.create_timer(0.05, self.publish_cmd)  # 20 Hz

        print(HELP)

    def publish_cmd(self):
        self.cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cmd)

    def stop(self):
        self.active_keys.clear()
        self.cmd = TwistStamped()
        self.cmd.header.frame_id = 'base_link'

    def call_trigger_service(self, client, name):
        self.stop()

        if not client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f'Service {name} is not available.')
            return

        future = client.call_async(Trigger.Request())

        def done_callback(fut):
            try:
                response = fut.result()
                if response.success:
                    self.get_logger().info(f'{name}: {response.message}')
                else:
                    self.get_logger().warn(f'{name} failed: {response.message}')
            except Exception as exc:
                self.get_logger().error(f'{name} call failed: {exc}')

        future.add_done_callback(done_callback)

    def update_cmd_from_active_keys(self):
        self.cmd = TwistStamped()
        self.cmd.header.frame_id = 'base_link'

        vx = (
            self.speed_xy * int('w' in self.active_keys)
            - self.speed_xy * int('s' in self.active_keys)
        )
        vy = (
            self.speed_xy * int('a' in self.active_keys)
            - self.speed_xy * int('d' in self.active_keys)
        )
        xy_norm = math.hypot(vx, vy)
        if xy_norm > PX4_MAX_XY_SPEED and xy_norm > 1e-6:
            scale = PX4_MAX_XY_SPEED / xy_norm
            vx *= scale
            vy *= scale

        self.cmd.twist.linear.x = vx
        self.cmd.twist.linear.y = vy
        self.cmd.twist.linear.z = (
            self.speed_z_up * int('r' in self.active_keys)
            - self.speed_z_down * int('f' in self.active_keys)
        )
        self.cmd.twist.angular.z = (
            self.yaw_rate * int('q' in self.active_keys)
            - self.yaw_rate * int('e' in self.active_keys)
        )

    def increase_speed(self):
        self.speed_xy = min(self.speed_xy + 0.8, PX4_MAX_XY_SPEED)
        self.speed_z_up = min(self.speed_z_up + 0.2, PX4_MAX_Z_SPEED_UP)
        self.speed_z_down = min(self.speed_z_down + 0.1, PX4_MAX_Z_SPEED_DOWN)
        self.update_cmd_from_active_keys()
        self.get_logger().info(
            f'speed_xy={self.speed_xy:.1f}, '
            f'speed_z_up={self.speed_z_up:.1f}, '
            f'speed_z_down={self.speed_z_down:.1f}'
        )

    def decrease_speed(self):
        self.speed_xy = max(self.speed_xy - 0.8, 0.8)
        self.speed_z_up = max(self.speed_z_up - 0.2, 0.2)
        self.speed_z_down = max(self.speed_z_down - 0.1, 0.1)
        self.update_cmd_from_active_keys()
        self.get_logger().info(
            f'speed_xy={self.speed_xy:.1f}, '
            f'speed_z_up={self.speed_z_up:.1f}, '
            f'speed_z_down={self.speed_z_down:.1f}'
        )

    def increase_yaw_rate(self):
        self.yaw_rate = min(self.yaw_rate + 0.1, MAX_YAW_RATE)
        self.update_cmd_from_active_keys()
        self.get_logger().info(f'yaw_rate={self.yaw_rate:.1f}')

    def decrease_yaw_rate(self):
        self.yaw_rate = max(self.yaw_rate - 0.1, MIN_YAW_RATE)
        self.update_cmd_from_active_keys()
        self.get_logger().info(f'yaw_rate={self.yaw_rate:.1f}')

    def handle_key_event(self, key, value):
        if key in MOVEMENT_KEYS:
            if value == KEY_PRESS:
                self.active_keys.add(key)
                self.update_cmd_from_active_keys()
            elif value == KEY_RELEASE:
                self.active_keys.discard(key)
                self.update_cmd_from_active_keys()
            return False

        if value != KEY_PRESS:
            return False

        if key == '1':
            self.call_trigger_service(
                self.offboard_arm_client, '/uav/offboard_arm'
            )
            return False

        if key == '2':
            self.call_trigger_service(self.land_client, '/uav/land')
            return False

        if key == '3':
            self.call_trigger_service(self.disarm_client, '/uav/disarm')
            return False

        if key == '4':
            print(HELP)
            return False

        if key == 'up':
            self.increase_speed()
            return False

        if key == 'down':
            self.decrease_speed()
            return False

        if key == 'right':
            self.increase_yaw_rate()
            return False

        if key == 'left':
            self.decrease_yaw_rate()
            return False

        return key == 'x'


def main():
    rclpy.init()
    node = None
    keyboard = None
    terminal = None

    try:
        node = KeyboardCmdVel()
        terminal = TerminalMode()

        if is_ssh_session():
            keyboard = TerminalKeyboard()
            node.get_logger().info(
                'SSH session detected; using terminal keyboard fallback. '
                'Key release is approximated from key repeat timing.'
            )
        else:
            try:
                keyboard = LinuxKeyboard()
            except RuntimeError as exc:
                node.get_logger().warn(str(exc))
                node.get_logger().warn(
                    'Falling back to terminal keyboard input. '
                    'Key release is approximated from key repeat timing.'
                )
                keyboard = TerminalKeyboard()

        exit_requested = False

        while rclpy.ok() and not exit_requested:
            for key, value in keyboard.read_events(0.05):
                if node.handle_key_event(key, value):
                    exit_requested = True
                    break

            rclpy.spin_once(node, timeout_sec=0.0)

    finally:
        if terminal is not None:
            terminal.restore()

        if keyboard is not None:
            keyboard.close()

        if node is not None:
            node.stop()
            node.publish_cmd()
            node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
