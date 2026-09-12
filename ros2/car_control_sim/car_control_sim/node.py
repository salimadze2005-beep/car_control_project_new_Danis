import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from car_control_core.core import Parameters, Bicycle, circle_track, world_to_cones, Command
from car_control_core.fsds import (track_from_message, vehicle_cones, fill_command,
                                   LongitudinalController, speed_limited_command,
                                   validate_host)
from car_control_core.session import Session
from car_control_core.sensor import ConeSensor, SensorParameters


class ControllerNode(Node):
    def __init__(self):
        super().__init__('car_controller')
        backend = self.declare_parameter('backend', 'lightweight').value
        if backend not in ('lightweight', 'fsds'):
            raise ValueError('backend must be lightweight or fsds')
        self.backend = backend
        config_path = self.declare_parameter('controller_config', '').value
        values = json.loads(Path(config_path).read_text()) if config_path else {}
        parameters = Parameters(**values)
        enabled = self.declare_parameter('auto_start', False).value
        data_timeout = float(self.declare_parameter('data_timeout', 0.4).value)
        self.session = Session(parameters, enabled=enabled, timeout=data_timeout,
                               require_go=backend == 'fsds')
        self.host = validate_host(self.declare_parameter('host', 'localhost').value)
        self.tick_period = float(self.declare_parameter('tick_period', 0.02).value)
        if not 0.01 <= self.tick_period <= 10.0:
            raise ValueError('tick_period must be between 0.01 and 10 seconds')
        self.create_service(SetBool, '/car/enable', self.enable)
        self.status = self.create_publisher(String, '/car/status', 1)
        self.commands = self.create_publisher(String, '/car/command_debug', 1)
        self.odom_pub = self.create_publisher(Odometry, '/car/sim/odom', 1)
        self.markers = self.create_publisher(MarkerArray, '/car/sim/markers', 1)
        self.track = []
        self.last_reason = None
        self.last_wall = time.monotonic()
        if backend == 'fsds':
            sensor_parameters = SensorParameters(
                rate_hz=float(self.declare_parameter('sensor_rate_hz', 0.0).value),
                latency_s=float(self.declare_parameter('sensor_latency_s', 0.0).value),
                dropout_probability=float(self.declare_parameter('sensor_dropout_probability', 0.0).value),
                lateral_std_m=float(self.declare_parameter('sensor_lateral_std_m', 0.0).value),
                depth_std_m=float(self.declare_parameter('sensor_depth_std_m', 0.0).value),
                depth_relative_std=float(self.declare_parameter('sensor_depth_relative_std', 0.0).value),
                camera_offset_x_m=float(self.declare_parameter('sensor_camera_offset_x_m', 0.0).value),
                camera_offset_z_m=float(self.declare_parameter('sensor_camera_offset_z_m', 0.0).value),
                min_depth_m=parameters.min_depth,
                max_depth_m=parameters.max_depth,
                max_per_color=int(self.declare_parameter('sensor_max_per_color', 0).value),
                seed=int(self.declare_parameter('sensor_seed', 0).value),
            )
            self.sensor = ConeSensor(sensor_parameters)
            self.sensor_fov = math.radians(float(self.declare_parameter('sensor_fov_deg', 90.0).value))
            if not 0 < self.sensor_fov <= math.pi:
                raise ValueError('sensor_fov_deg must be in (0, 180]')
            self.fsds_speed_mps = 0.
            self.fsds_max_speed_mps = float(self.declare_parameter('fsds_max_speed_mps', 2.0).value)
            self.fsds_speed_brake = float(self.declare_parameter('fsds_speed_brake', 0.25).value)
            self.fsds_throttle_scale = float(self.declare_parameter('fsds_throttle_scale', 0.2).value)
            self.fsds_speed_soft_zone_mps = float(self.declare_parameter('fsds_speed_soft_zone_mps', 0.0).value)
            self.fsds_speed_brake_margin_mps = float(self.declare_parameter('fsds_speed_brake_margin_mps', 0.0).value)
            self.fsds_overspeed_brake_zone_mps = float(
                self.declare_parameter('fsds_overspeed_brake_zone_mps', 0.0).value)
            self.fsds_speed_kp = float(self.declare_parameter('fsds_speed_kp', 0.08).value)
            self.fsds_speed_ki = float(self.declare_parameter('fsds_speed_ki', 0.04).value)
            self.fsds_speed_integral_limit = float(
                self.declare_parameter('fsds_speed_integral_limit', 3.0).value)
            self.fsds_speed_brake_gain = float(
                self.declare_parameter('fsds_speed_brake_gain', 1.0).value)
            self.fsds_speed_breakaway_throttle = float(
                self.declare_parameter('fsds_speed_breakaway_throttle', 0.20).value)
            self.fsds_longitudinal_mode = str(
                self.declare_parameter('fsds_longitudinal_mode', 'legacy').value)
            if self.fsds_longitudinal_mode not in ('legacy', 'pi'):
                raise ValueError('fsds_longitudinal_mode must be legacy or pi')
            self.speed_controller = LongitudinalController(
                kp=self.fsds_speed_kp, ki=self.fsds_speed_ki,
                integral_limit=self.fsds_speed_integral_limit,
                brake_gain=self.fsds_speed_brake_gain,
                maximum_brake=self.fsds_speed_brake,
                breakaway_throttle=self.fsds_speed_breakaway_throttle)
            if (not 0.1 <= self.fsds_max_speed_mps <= 15.0
                    or not 0 <= self.fsds_speed_brake <= 1
                    or not 0 <= self.fsds_throttle_scale <= 1
                    or self.fsds_speed_soft_zone_mps < 0
                    or self.fsds_overspeed_brake_zone_mps < 0
                    or not 0 <= self.fsds_speed_brake_margin_mps < self.fsds_max_speed_mps):
                raise ValueError('FSDS speed limiter configuration is invalid')
            # Optional dependency: no fs_msgs import in lightweight mode.
            from fs_msgs.msg import Track, GoSignal, ControlCommand
            self.command_type = ControlCommand
            self.command_pub = self.create_publisher(ControlCommand, '/fsds/control_command', 1)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Track, '/fsds/testing_only/track', self.track_received, latched)
            self.create_subscription(Odometry, '/fsds/testing_only/odom', self.odom_received, 10)
            self.create_subscription(GoSignal, '/fsds/signal/go', self.go_received, 1)
        else:
            self.car, self.track = Bicycle(), circle_track()
        # Explicit steady clock: watchdog must fire even when FSDS /clock stops.
        self.timer = self.create_timer(self.tick_period, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.add_on_set_parameters_callback(self.validate_runtime_speed)

    def validate_runtime_speed(self, parameters):
        for parameter in parameters:
            if parameter.name == 'fsds_max_speed_mps':
                value = parameter.value
                if (self.backend != 'fsds' or isinstance(value, bool)
                        or not isinstance(value, (int, float))
                        or not math.isfinite(value) or not 0.1 <= value <= 15.0
                        or value <= self.fsds_speed_brake_margin_mps):
                    return SetParametersResult(successful=False, reason=
                        'FSDS target must be finite, 0.1..15.0 m/s and above brake margin')
            if parameter.name == 'fsds_throttle_scale':
                value = parameter.value
                if (self.backend != 'fsds' or isinstance(value, bool)
                        or not isinstance(value, (int, float))
                        or not math.isfinite(value) or not 0.0 <= value <= 1.0):
                    return SetParametersResult(
                        successful=False,
                        reason='FSDS throttle scale must be finite and within 0..1')
        return SetParametersResult(successful=True)

    def enable(self, request, response):
        self.session.enable(request.data)
        response.success, response.message = True, 'enabled' if request.data else 'disabled'
        return response

    def track_received(self, msg):
        self.track = track_from_message(msg)
        self.sensor.reset()
        self.get_logger().info('FSDS track received: %d known cones' % len(self.track))

    def odom_received(self, msg):
        if not self.track:
            return
        try:
            velocity = msg.twist.twist.linear
            self.fsds_speed_mps = math.hypot(velocity.x, velocity.y)
            cones = vehicle_cones(self.track, msg, self.session.core.p.max_depth,
                                  self.sensor_fov)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.sensor.capture(cones, stamp, time.monotonic())
        except ValueError as error:
            self.session.sample = None
            self.get_logger().error(str(error))

    def go_received(self, _):
        # GoSignal.header is mission start, NOT heartbeat time.
        self.session.go_time = time.monotonic()

    def tick(self):
        now = time.monotonic()
        source_now = self.get_clock().now().nanoseconds / 1e9
        if self.backend == 'lightweight':
            cones = world_to_cones(self.track, self.car.x, self.car.y, self.car.yaw, self.session.core.p.max_depth)
            self.session.observe(cones, source_now, now)
        else:
            observation = self.sensor.ready(now)
            if observation is not None:
                self.session.observe(observation.cones, observation.source_stamp,
                                     observation.received_stamp)
        command = self.session.tick(now, source_now)
        transport_command = command
        if self.backend == 'fsds':
            self.fsds_max_speed_mps = float(self.get_parameter('fsds_max_speed_mps').value)
            self.fsds_throttle_scale = float(
                self.get_parameter('fsds_throttle_scale').value)
            if self.fsds_longitudinal_mode == 'pi':
                transport_command = self.speed_controller.step(
                    command, self.fsds_speed_mps, self.fsds_max_speed_mps,
                    self.fsds_throttle_scale, self.tick_period)
            else:
                transport_command = speed_limited_command(
                    command, self.fsds_speed_mps, self.fsds_max_speed_mps,
                    self.fsds_speed_brake, self.fsds_throttle_scale,
                    self.fsds_speed_soft_zone_mps,
                    self.fsds_speed_brake_margin_mps,
                    self.fsds_overspeed_brake_zone_mps)
            self.command_pub.publish(fill_command(self.command_type(), transport_command, self.get_clock().now().to_msg()))
        else:
            self.car.step(command, min(max(now-self.last_wall, 0.), 0.1))
            self.publish_model()
        self.last_wall = now
        self.commands.publish(String(data=json.dumps(vars(transport_command))))
        status = {'backend': self.backend, 'host': self.host,
                  'enabled': self.session.enabled, 'reason': self.session.reason}
        if self.backend == 'fsds':
            status.update({'speed_mps': self.fsds_speed_mps,
                           'target_speed_mps': self.fsds_max_speed_mps,
                           'throttle_scale': self.fsds_throttle_scale,
                           'visible_cones': len(self.session.sample[0])
                           if self.session.sample else 0,
                           'perception': 'ground_truth',
                           'longitudinal_mode': self.fsds_longitudinal_mode})
        self.status.publish(String(data=json.dumps(status)))
        if self.last_reason != self.session.reason:
            self.get_logger().info('Control state: ' + self.session.reason)
            self.last_reason = self.session.reason

    def publish_model(self):
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'map', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.car.x, self.car.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.car.yaw/2), math.cos(self.car.yaw/2)
        odom.twist.twist.linear.x = self.car.speed
        self.odom_pub.publish(odom)
        markers = MarkerArray()
        for i, (x, y, color) in enumerate(self.track + [(self.car.x, self.car.y, 'car')]):
            m = Marker()
            m.header.frame_id, m.header.stamp = 'map', stamp
            m.ns, m.id, m.type, m.action = 'simulation', i, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.orientation.w = x, y, 1.
            m.scale.x = m.scale.y = 0.15 if color != 'car' else 0.32
            m.scale.z, m.color.a = 0.25, 1.
            m.color.r, m.color.g, m.color.b = float(color == 'yellow'), float(color != 'blue'), float(color == 'blue')
            markers.markers.append(m)
        self.markers.publish(markers)

    def stop(self):
        if self.backend == 'fsds' and rclpy.ok():
            self.command_pub.publish(fill_command(self.command_type(), Command(), self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
