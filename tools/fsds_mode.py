#!/usr/bin/env python3
"""Safely switch one running FSDS stack between keyboard and autopilot."""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


def wait_briefly(node, seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))


def call_set_bool(node, service_name, value, timeout=8.0, retries=4):
    """Survive the upstream bridge disappearing while launch respawns it."""
    last_error = 'service unavailable'
    for attempt in range(retries):
        client = node.create_client(SetBool, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                last_error = 'service unavailable'
                continue
            future = client.call_async(SetBool.Request(data=value))
            deadline = time.monotonic() + timeout
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if not future.done():
                last_error = 'request timed out'
                continue
            response = future.result()
            if response is not None and response.success:
                return response.message
            last_error = 'request rejected: %s' % getattr(
                response, 'message', 'no response')
        except Exception as error:
            last_error = str(error)
        finally:
            node.destroy_client(client)
        if attempt + 1 < retries:
            wait_briefly(node, 0.5 * (attempt + 1))
    raise RuntimeError(
        '%s failed after %d attempts (%s). Ensure FSDS is running with a loaded '
        'track; launch will respawn the bridge automatically.' %
        (service_name, retries, last_error))


def wait_for_status(node, predicate, timeout):
    latest = {'value': None}

    def received(message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict):
                latest['value'] = value
        except (TypeError, ValueError):
            pass

    subscription = node.create_subscription(String, '/car/status', received, 10)
    try:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if latest['value'] is not None and predicate(latest['value']):
                return latest['value']
        return latest['value']
    finally:
        node.destroy_subscription(subscription)


def ready_for_enable(status):
    return (status.get('backend') == 'fsds'
            and status.get('enabled') is False
            and status.get('reason') == 'disabled'
            and status.get('visible_cones', 0) > 0)


def set_mode(node, mode):
    # Disable first: no transition can accidentally preserve an old throttle.
    call_set_bool(node, '/car/enable', False, timeout=5.0, retries=2)
    if mode == 'manual':
        call_set_bool(node, '/fsds/manual_mode', True)
        return 'MANUAL: FSDS keyboard owns the car'
    if mode == 'stop':
        call_set_bool(node, '/fsds/manual_mode', False)
        return 'STOP: API owns the car and controller holds full brake'
    last_status = None
    for _ in range(2):
        call_set_bool(node, '/car/enable', False, timeout=5.0, retries=2)
        call_set_bool(node, '/fsds/manual_mode', False)
        last_status = wait_for_status(node, ready_for_enable, 10.0)
        if last_status is None or not ready_for_enable(last_status):
            continue
        call_set_bool(node, '/car/enable', True, timeout=5.0, retries=2)
        last_status = wait_for_status(
            node, lambda value: (value.get('enabled') is True
                                 and value.get('reason') in (
                                     'running', 'connection_fault_reenable_required',
                                     'no_usable_cones', 'invalid_command', 'finished')),
            5.0)
        if last_status is not None and last_status.get('reason') == 'running':
            return 'AUTO: shared controller owns the car'
        if (last_status is None
                or last_status.get('reason') != 'connection_fault_reenable_required'):
            break
    call_set_bool(node, '/car/enable', False, timeout=5.0, retries=2)
    reason = 'no fresh cone/odometry data' if last_status is None else last_status.get(
        'reason', 'unknown')
    raise RuntimeError(
        'AUTOPILOT stayed disabled (%s). Keep FSDS open with a loaded track, '
        'point the car along blue/yellow cones, then try AUTOPILOT again.' % reason)


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
