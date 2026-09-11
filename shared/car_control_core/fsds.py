"""FSDS v2.2.0 contracts. No ROS imports and no RPC implementation."""
import ipaddress
import math
import re
from .core import Command, clamp, world_to_cones

COLORS = {0: 'blue', 1: 'yellow', 2: 'orange', 3: 'orange'}


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

def speed_limited_command(command, speed, max_speed, brake):
    """Apply FSDS drivetrain limits without changing controller-core math."""
    if not all(math.isfinite(value) for value in (speed, max_speed, brake)) or max_speed <= 0:
        return Command()
    command = map_command(command)
    if speed >= max_speed:
        return Command(0., command.steering, clamp(brake, 0., 1.))
    return command


def fill_command(message, command, stamp):
    command = map_command(command)
    message.header.stamp = stamp
    message.throttle, message.steering, message.brake = command.throttle, command.steering, command.brake
    return message
