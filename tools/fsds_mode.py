#!/usr/bin/env python3
"""Safely switch one running FSDS stack between keyboard and autopilot."""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


def call_set_bool(node, service_name, value, timeout=5.0):
    client = node.create_client(SetBool, service_name)
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError('%s is unavailable; rebuild and start fsds_drive.launch.py' % service_name)
    future = client.call_async(SetBool.Request(data=value))
    deadline = time.monotonic() + timeout
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not future.done():
        raise RuntimeError('%s timed out' % service_name)
    response = future.result()
    if response is None or not response.success:
        raise RuntimeError('%s rejected request: %s' %
                           (service_name, getattr(response, 'message', 'no response')))
    return response.message


def set_mode(node, mode):
    # Disable first: no transition can accidentally preserve an old throttle.
    call_set_bool(node, '/car/enable', False)
    if mode == 'manual':
        call_set_bool(node, '/fsds/manual_mode', True)
        return 'MANUAL: FSDS keyboard owns the car'
    if mode == 'stop':
        call_set_bool(node, '/fsds/manual_mode', False)
        return 'STOP: API owns the car and controller holds full brake'
    call_set_bool(node, '/fsds/manual_mode', False)
    call_set_bool(node, '/car/enable', True)
    return 'AUTO: shared controller owns the car'


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=('manual', 'auto', 'stop'))
    args = parser.parse_args(argv)
    rclpy.init()
    node = Node('fsds_mode_switch')
    try:
        print(set_mode(node, args.mode))
        return 0
    except Exception as error:
        print('MODE SWITCH FAILED:', error)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
