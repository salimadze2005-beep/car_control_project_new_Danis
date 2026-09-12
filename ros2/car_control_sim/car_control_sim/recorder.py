"""Record the FSDS front camera and synchronized driving telemetry."""
import csv
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


class RunRecorder(Node):
    def __init__(self):
        super().__init__('fsds_run_recorder')
        output = self.declare_parameter('output_dir', '').value
        if not output:
            raise ValueError('output_dir is required')
        self.output = Path(output).expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.camera_topic = self.declare_parameter(
            'camera_topic', '/fsds/front/image_color').value
        self.fps = float(self.declare_parameter('video_fps', 15.0).value)
        if not 1 <= self.fps <= 120:
            raise ValueError('video_fps must be within 1..120')

        self.started = time.monotonic()
        self.writer = None
        self.video_size = None
        self.frames = 0
        self.samples = 0
        self.distance = 0.0
        self.maximum_speed = 0.0
        self.previous_position = None
        self.command = {'throttle': 0.0, 'steering': 0.0, 'brake': 1.0}
        self.status = {'enabled': False, 'reason': 'unknown',
                       'target_speed_mps': 0.0, 'visible_cones': 0}
        self.closed = False

        self.csv_handle = (self.output / 'telemetry.csv').open(
            'w', newline='', encoding='utf-8')
        self.csv = csv.DictWriter(self.csv_handle, fieldnames=[
            'elapsed_s', 'ros_stamp_s', 'x_m', 'y_m', 'speed_mps',
            'target_speed_mps', 'throttle', 'steering', 'brake',
            'enabled', 'reason', 'visible_cones'])
        self.csv.writeheader()

        camera_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, self.camera_topic, self.image_received,
                                 camera_qos)
        self.create_subscription(Odometry, '/fsds/testing_only/odom',
                                 self.odom_received, 20)
        self.create_subscription(String, '/car/command_debug',
                                 self.command_received, 10)
        self.create_subscription(String, '/car/status', self.status_received, 10)
        self.get_logger().info('Recording to %s' % self.output)

    @staticmethod
    def _json(message, previous):
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else previous
        except (TypeError, ValueError):
            return previous

    def command_received(self, message):
        self.command = self._json(message, self.command)

    def status_received(self, message):
        self.status = self._json(message, self.status)

    def image_received(self, message):
        if message.encoding.lower() != 'bgr8' or message.height <= 0 or message.width <= 0:
            self.get_logger().error('Expected non-empty bgr8 FSDS camera image')
            return
        row_bytes = message.width * 3
        if message.step < row_bytes or len(message.data) < message.step * message.height:
            self.get_logger().error('Invalid FSDS image dimensions')
            return
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step)[:, :row_bytes].reshape(
                message.height, message.width, 3)
        size = (message.width, message.height)
        if self.writer is None:
            self.video_size = size
            self.writer = cv2.VideoWriter(
                str(self.output / 'camera.mp4'),
                cv2.VideoWriter_fourcc(*'mp4v'), self.fps, size)
            if not self.writer.isOpened():
                self.writer = None
                raise RuntimeError('OpenCV could not open camera.mp4 writer')
        if size != self.video_size:
            image = cv2.resize(image, self.video_size)
        self.writer.write(image)
        self.frames += 1

    def odom_received(self, message):
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        point = (position.x, position.y)
        if self.previous_position is not None:
            segment = math.dist(self.previous_position, point)
            if math.isfinite(segment) and segment < 10.0:
                self.distance += segment
        self.previous_position = point
        self.maximum_speed = max(self.maximum_speed, speed)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.csv.writerow({
            'elapsed_s': '%.6f' % (time.monotonic() - self.started),
            'ros_stamp_s': '%.9f' % stamp,
            'x_m': '%.6f' % position.x, 'y_m': '%.6f' % position.y,
            'speed_mps': '%.6f' % speed,
            'target_speed_mps': self.status.get('target_speed_mps', ''),
            'throttle': self.command.get('throttle', ''),
            'steering': self.command.get('steering', ''),
            'brake': self.command.get('brake', ''),
            'enabled': self.status.get('enabled', ''),
            'reason': self.status.get('reason', ''),
            'visible_cones': self.status.get('visible_cones', '')})
        self.samples += 1
        if self.samples % 20 == 0:
            self.csv_handle.flush()

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.writer is not None:
            self.writer.release()
        self.csv_handle.flush()
        self.csv_handle.close()
        summary = {
            'duration_s': time.monotonic() - self.started,
            'distance_m': self.distance,
            'maximum_speed_mps': self.maximum_speed,
            'video_frames': self.frames,
            'telemetry_samples': self.samples,
            'camera_topic': self.camera_topic,
            'video_created': self.frames > 0}
        (self.output / 'summary.json').write_text(
            json.dumps(summary, indent=2), encoding='utf-8')
        self.get_logger().info('Recording complete: %s' % summary)


def main(args=None):
    rclpy.init(args=args)
    node = RunRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
