#!/usr/bin/env python3
"""Atomically tune speed and steering without enabling the car."""
import argparse
import math
import rclpy
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('speed', type=float, help='Target speed in m/s, 0.1..15.0')
    parser.add_argument('--throttle-scale', type=float,
                        help='Optional maximum FSDS throttle, 0..1')
    parser.add_argument('--steering-gain', type=float,
                        help='Optional steering proportional gain, 0..3')
    parser.add_argument('--steering-response', type=float,
                        help='Optional EMA response, 0.05..1 (higher is faster)')
    parser.add_argument('--steering-limit', type=float,
                        help='Optional maximum normalized steering, 0.1..1')
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not 0.1 <= args.speed <= 15.0:
        parser.error('speed must be finite and within 0.1..15.0 m/s')
    if (args.throttle_scale is not None
            and (not math.isfinite(args.throttle_scale)
                 or not 0 <= args.throttle_scale <= 1)):
        parser.error('throttle scale must be finite and within 0..1')
    steering_values = {
        'steering_gain': (args.steering_gain, 0.0, 3.0),
        'steering_response': (args.steering_response, 0.05, 1.0),
        'steering_limit': (args.steering_limit, 0.1, 1.0),
    }
    for name, (value, low, high) in steering_values.items():
        if value is not None and (not math.isfinite(value) or not low <= value <= high):
            parser.error('%s must be finite and within %.2f..%.2f' % (
                name.replace('_', ' '), low, high))
    rclpy.init()
    node = rclpy.create_node('fsds_speed_setting')
    try:
        client = node.create_client(SetParametersAtomically,
                                   '/car_controller/set_parameters_atomically')
        if not client.wait_for_service(timeout_sec=5.):
            raise RuntimeError('Controller unavailable; start fsds_drive.launch.py')
        parameters = [
            Parameter('fsds_max_speed_mps', value=args.speed).to_parameter_msg()]
        if args.throttle_scale is not None:
            parameters.append(Parameter(
                'fsds_throttle_scale', value=args.throttle_scale).to_parameter_msg())
        for name, (value, _, _) in steering_values.items():
            if value is not None:
                parameters.append(Parameter(name, value=value).to_parameter_msg())
        request = SetParametersAtomically.Request(parameters=parameters)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done():
            raise RuntimeError('Controller did not acknowledge speed')
        result = future.result().result
        if not result.successful:
            raise RuntimeError(result.reason)
        message = 'Target applied: %.2f m/s (%.2f km/h)' % (
            args.speed, args.speed * 3.6)
        if args.throttle_scale is not None:
            message += ', throttle limit %.2f' % args.throttle_scale
        if args.steering_gain is not None:
            message += ', steering gain %.2f' % args.steering_gain
        if args.steering_response is not None:
            message += ', response %.2f' % args.steering_response
        if args.steering_limit is not None:
            message += ', steering limit %.2f' % args.steering_limit
        print(message)
        return 0
    except Exception as error:
        print('Speed not applied:', error)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
