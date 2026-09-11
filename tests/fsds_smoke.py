#!/usr/bin/env python3
"""Operator-invoked FSDS motion smoke test; requires a running launch + GUI."""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from fs_msgs.msg import ControlCommand


class Probe(Node):
    def __init__(self):
        super().__init__('fsds_motion_probe')
        self.odometry = []
        self.commands = []
        self.statuses = []
        self.latest_command = None
        self.trace = []
        self.create_subscription(
            Odometry, '/fsds/testing_only/odom', self.on_odometry, 20)
        self.create_subscription(
            ControlCommand, '/fsds/control_command', self.on_command, 20)
        self.create_subscription(
            String, '/car/status', lambda msg: self.statuses.append(json.loads(msg.data)), 20)
        self.enable_client = self.create_client(SetBool, '/car/enable')

    def on_command(self, message):
        self.latest_command = message
        self.commands.append(message)

    def on_odometry(self, message):
        self.odometry.append(message)
        if self.latest_command is not None:
            self.trace.append((time.monotonic(), speed(message),
                               self.latest_command.throttle,
                               self.latest_command.brake))

    def spin_until(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not condition():
            rclpy.spin_once(self, timeout_sec=0.05)
        return condition()

    def set_enabled(self, enabled, timeout=3.0):
        if not self.enable_client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('/car/enable service is unavailable')
        future = self.enable_client.call_async(SetBool.Request(data=enabled))
        if not self.spin_until(future.done, timeout):
            raise RuntimeError('/car/enable call timed out')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('/car/enable rejected the request')


def position(message):
    point = message.pose.pose.position
    return point.x, point.y


def speed(message):
    velocity = message.twist.twist.linear
    return math.hypot(velocity.x, velocity.y)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=8.0)
    parser.add_argument('--minimum-distance', type=float, default=0.5)
    parser.add_argument(
        '--maximum-speed', type=float, default=2.5,
        help='runaway ceiling, not the FSDS governor target (default: 2.5 m/s)')
    parser.add_argument('--maximum-command-throttle', type=float, default=0.21)
    parser.add_argument('--trace', action='store_true')
    args = parser.parse_args(argv)
    if (args.seconds <= 0 or args.minimum_distance < 0 or args.maximum_speed <= 0
            or not 0 <= args.maximum_command_throttle <= 1):
        parser.error('invalid duration, distance, speed or throttle bound')

    rclpy.init()
    probe = Probe()
    failure = None
    try:
        if not probe.spin_until(
                lambda: probe.odometry and probe.commands and probe.statuses, 8.0):
            raise RuntimeError('Missing odom, command or status before enable')
        if probe.statuses[-1].get('enabled'):
            raise RuntimeError('Controller must be disabled before smoke test')
        if probe.commands[-1].throttle != 0.0 or probe.commands[-1].brake != 1.0:
            raise RuntimeError('Disabled controller is not publishing full brake')

        start = position(probe.odometry[-1])
        start_index = len(probe.odometry)
        probe.set_enabled(True)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.05)
        samples = probe.odometry[start_index:]
        if not samples:
            raise RuntimeError('No odometry while enabled')
        end = position(samples[-1])
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        maximum_speed = max(speed(message) for message in samples)
        maximum_throttle = max(command.throttle for command in probe.commands)
        reasons = sorted({status.get('reason', '') for status in probe.statuses})
        print('distance_m=%.3f max_speed_mps=%.3f max_throttle=%.3f reasons=%s' %
              (distance, maximum_speed, maximum_throttle, ','.join(reasons)))
        if args.trace and probe.trace:
            origin = probe.trace[0][0]
            for stamp, measured_speed, throttle, brake in probe.trace:
                print('trace t=%.3f speed=%.3f throttle=%.3f brake=%.3f' %
                      (stamp - origin, measured_speed, throttle, brake))
        if distance < args.minimum_distance:
            raise RuntimeError('Vehicle did not move far enough')
        if maximum_speed > args.maximum_speed:
            raise RuntimeError('Vehicle exceeded smoke-test speed ceiling')
        if maximum_throttle > args.maximum_command_throttle + 1e-6:
            raise RuntimeError('FSDS throttle calibration was exceeded')
    except Exception as error:
        failure = error
    finally:
        try:
            probe.set_enabled(False)
            probe.spin_until(
                lambda: bool(probe.commands) and probe.commands[-1].brake == 1.0, 2.0)
            if not probe.commands or probe.commands[-1].brake != 1.0:
                failure = failure or RuntimeError('Final brake command was not observed')
        except Exception as error:
            failure = failure or error
        probe.destroy_node()
        rclpy.shutdown()
    if failure:
        print('FSDS smoke FAIL:', failure)
        return 1
    print('FSDS motion and final disable/brake PASS')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
