#!/usr/bin/env python3
"""Change the running FSDS governor target without enabling the car."""
import argparse
import math
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('speed', type=float, help='Target speed in m/s, 0.1..5.0')
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.1 <= args.speed <= 5.0:
        parser.error('speed must be finite and within 0.1..5.0 m/s')
    rclpy.init()
    node = rclpy.create_node('fsds_speed_setting')
    try:
        client = node.create_client(SetParametersAtomically,
                                   '/car_controller/set_parameters_atomically')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Controller unavailable; start fsds_drive.launch.py')
        request = SetParametersAtomically.Request(parameters=[
            Parameter('fsds_max_speed_mps', value=args.speed).to_parameter_msg()])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done():
            raise RuntimeError('Controller did not acknowledge speed')
        result = future.result().result
        if not result.successful:
            raise RuntimeError(result.reason)
        print('Target applied: %.2f m/s (%.2f km/h)' % (args.speed, args.speed * 3.6))
        return 0
    except Exception as error:
        print('Speed not applied:', error)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
