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
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from car_control_core.core import Parameters, Bicycle, circle_track, world_to_cones, Command
from car_control_core.fsds import track_from_message, vehicle_cones, fill_command, speed_limited_command, validate_host
from car_control_core.session import Session


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
        self.session = Session(parameters, enabled=enabled, require_go=backend == 'fsds')
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
            self.fsds_speed_mps = 0.
            self.fsds_max_speed_mps = float(self.declare_parameter('fsds_max_speed_mps', 2.0).value)
            self.fsds_speed_brake = float(self.declare_parameter('fsds_speed_brake', 0.25).value)
            if not self.fsds_max_speed_mps > 0 or not 0 <= self.fsds_speed_brake <= 1:
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

    def enable(self, request, response):
        self.session.enable(request.data)
        response.success, response.message = True, 'enabled' if request.data else 'disabled'
        return response

    def track_received(self, msg):
        self.track = track_from_message(msg)
        self.get_logger().info('FSDS track received: %d known cones' % len(self.track))

    def odom_received(self, msg):
        if not self.track:
            return
        try:
            velocity = msg.twist.twist.linear
            self.fsds_speed_mps = math.hypot(velocity.x, velocity.y)
            cones = vehicle_cones(self.track, msg, self.session.core.p.max_depth)
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.session.observe(cones, stamp, time.monotonic())
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
        command = self.session.tick(now, source_now)
        transport_command = command
        if self.backend == 'fsds':
            transport_command = speed_limited_command(
                command, self.fsds_speed_mps, self.fsds_max_speed_mps, self.fsds_speed_brake)
            self.command_pub.publish(fill_command(self.command_type(), transport_command, self.get_clock().now().to_msg()))
        else:
            self.car.step(command, min(max(now-self.last_wall, 0.), 0.1))
            self.publish_model()
        self.last_wall = now
        self.commands.publish(String(data=json.dumps(vars(transport_command))))
        self.status.publish(String(data=json.dumps({'backend': self.backend, 'host': self.host,
                            'enabled': self.session.enabled, 'reason': self.session.reason})))
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
