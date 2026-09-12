"""FSDS v2.2.0 contracts. No ROS imports and no RPC implementation."""
import ipaddress
import math
import re
from .core import Command, clamp, world_to_cones

COLORS = {0: 'blue', 1: 'yellow', 2: 'orange', 3: 'orange'}


class LongitudinalController:
    """PI speed loop for the FSDS actuator adapter."""
    def __init__(self, kp=0.08, ki=0.04, integral_limit=3.0,
                 brake_gain=1.0, maximum_brake=1.0,
                 breakaway_throttle=0.20, breakaway_speed=0.20):
        values = (kp, ki, integral_limit, brake_gain, maximum_brake,
                  breakaway_throttle, breakaway_speed)
        if (not all(math.isfinite(value) for value in values)
                or kp < 0 or ki < 0 or integral_limit < 0 or brake_gain < 0
                or not 0 <= maximum_brake <= 1
                or not 0 <= breakaway_throttle <= 1 or breakaway_speed < 0):
            raise ValueError('Invalid longitudinal controller configuration')
        self.kp, self.ki = kp, ki
        self.integral_limit = integral_limit
        self.brake_gain, self.maximum_brake = brake_gain, maximum_brake
        self.breakaway_throttle, self.breakaway_speed = (
            breakaway_throttle, breakaway_speed)
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def step(self, command, speed, target_speed, throttle_scale, dt):
        values = (speed, target_speed, throttle_scale, dt)
        if (not all(math.isfinite(value) for value in values)
                or speed < 0 or target_speed <= 0
                or not 0 <= throttle_scale <= 1 or dt <= 0):
            self.reset()
            return Command()
        command = map_command(command)
        if command.brake > 0 or command.throttle <= 0:
            self.reset()
            return command

        error = target_speed - speed
        if error < 0:
            self.integral = max(0.0, self.integral + error * dt)
            braking = clamp(-error * self.brake_gain, 0.0,
                            self.maximum_brake)
            return Command(0.0, command.steering, braking)

        candidate_integral = clamp(self.integral + error * dt, 0.0,
                                   self.integral_limit)
        # Do not wind the integrator up while the actuator is already at its
        # configured ceiling.  FSDS accelerates quickly even at low throttle;
        # accumulated error would otherwise keep full throttle applied until
        # the target and create a repeated accelerate/brake cycle.
        candidate_throttle = self.kp * error + self.ki * candidate_integral
        if candidate_throttle < throttle_scale:
            self.integral = candidate_integral
        throttle = self.kp * error + self.ki * self.integral
        if speed < self.breakaway_speed and error > 0.05:
            throttle = max(throttle, self.breakaway_throttle)
        throttle = min(throttle, throttle_scale) * command.throttle
        return Command(throttle, command.steering, 0.0)


def validate_host(host):
    host = str(host).strip()
    if not host or len(host) > 253:
        raise ValueError('FSDS host must be localhost, an IP address or a DNS hostname')
    try:
        ipaddress.ip_address(host)
        return host
    except ValueError:
        pass
    if not all(re.fullmatch(r'[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?', part)
               for part in host.split('.')):
        raise ValueError('Use a hostname only: no URL, port, whitespace or shell arguments')
    return host


def track_from_message(message):
    """Upstream already emits metres, origin at vehicle start, ENU axes."""
    track = []
    for cone in message.track:
        x, y, z = cone.location.x, cone.location.y, cone.location.z
        color = COLORS.get(cone.color)
        if color is not None and all(math.isfinite(v) for v in (x, y, z)):
            track.append((x, y, color))
    return track


def pose_from_message(message, map_frame='fsds/map', vehicle_frame='fsds/FSCar'):
    if message.header.frame_id != map_frame or message.child_frame_id != vehicle_frame:
        raise ValueError('Unexpected FSDS odometry frames')
    p, q = message.pose.pose.position, message.pose.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
        raise ValueError('Non-finite odometry')
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-6 or abs(norm - 1) > 0.01:
        raise ValueError('Invalid orientation quaternion')
    x, y, z, w = (v / norm for v in (q.x, q.y, q.z, q.w))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return p.x, p.y, yaw


def vehicle_cones(track, odom, max_depth, fov=math.pi / 2):
    return world_to_cones(track, *pose_from_message(odom), max_depth, fov)


def map_command(command):
    """Normalized core steering is already +right like FSDS. Brake wins."""
    if not all(math.isfinite(v) for v in (command.throttle, command.steering, command.brake)):
        return Command()
    throttle = clamp(command.throttle, 0., 1.)
    steering = clamp(command.steering, -1., 1.)
    brake = clamp(command.brake, 0., 1.)
    return Command(0. if brake > 0 else throttle, steering, brake)


def speed_limited_command(command, speed, max_speed, brake, throttle_scale=1.0,
                          soft_zone=0.0, brake_margin=0.0,
                          overspeed_brake_zone=0.0):
    """Apply FSDS actuator calibration without changing controller-core math."""
    values = (speed, max_speed, brake, throttle_scale, soft_zone,
              brake_margin, overspeed_brake_zone)
    if (not all(math.isfinite(value) for value in values) or max_speed <= 0
            or not 0 <= throttle_scale <= 1 or soft_zone < 0
            or overspeed_brake_zone < 0
            or not 0 <= brake_margin < max_speed):
        return Command()
    command = map_command(command)
    if command.brake > 0:
        return command
    if brake_margin > 0 and speed >= max_speed - brake_margin:
        return Command(0., command.steering, clamp(brake, 0., 1.))
    if speed >= max_speed:
        if overspeed_brake_zone > 0:
            braking = clamp((speed - max_speed) / overspeed_brake_zone, 0., 1.)
            return Command(0., command.steering, clamp(brake, 0., 1.) * braking)
        return Command(0., command.steering, clamp(brake, 0., 1.))
    throttle = command.throttle * throttle_scale
    if soft_zone > 0:
        start = max(0.0, max_speed - soft_zone)
        width = max_speed - start
        if speed > start and width > 0:
            throttle *= clamp((max_speed - speed) / width, 0.0, 1.0)
    return Command(throttle, command.steering, command.brake)


def fill_command(message, command, stamp):
    command = map_command(command)
    message.header.stamp = stamp
    message.throttle, message.steering, message.brake = command.throttle, command.steering, command.brake
    return message
