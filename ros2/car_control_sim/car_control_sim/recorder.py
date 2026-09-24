"""Recoverable camera recording and timestamped FSDS experiment diagnostics."""
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import cv2
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.msg import Log, ParameterEvent
from rosgraph_msgs.msg import Clock as ClockMessage
from rosidl_runtime_py.convert import message_to_ordereddict

from car_control_sim.recording import assemble_video, decode_image


class RunRecorder(Node):
    def __init__(self):
        super().__init__('fsds_run_recorder')
        output = self.declare_parameter('output_dir', '').value
        if not output:
            raise ValueError('output_dir is required')
        self.output = Path(output).expanduser().resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        # Never truncate a previous experiment.
        self.events = (self.output / 'events.jsonl').open('x', encoding='utf-8')
        (self.output / 'frames').mkdir(exist_ok=True)
        self.camera_topic = self.declare_parameter('camera_topic', '/fsds/front/image_color').value
        self.fps = float(self.declare_parameter('video_fps', 15.).value)
        if not math.isfinite(self.fps) or not 1 <= self.fps <= 120:
            raise ValueError('video_fps must be within 1..120')
        self.started = time.monotonic()
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.frames = []
        self.samples = self.invalid_images = self.invalid_odom = self.resets = 0
        self.distance = self.maximum_speed = self.error_squared = 0.
        self.error_samples = 0
        self.previous_position = None
        self.command, self.status = {}, {}
        self.command_time = self.status_time = self.odom_time = None
        self.last_image_time = self.last_image_stamp = self.last_clock = None
        self.max_camera_gap = 0.
        self.closed = False
        self.csv_handle = (self.output / 'telemetry.csv').open('x', newline='', encoding='utf-8')
        self.csv = csv.DictWriter(self.csv_handle, fieldnames=[
            'elapsed_s', 'ros_stamp_s', 'x_m', 'y_m', 'z_m', 'yaw_rad',
            'vx_mps', 'vy_mps', 'yaw_rate_radps', 'speed_mps', 'target_speed_mps',
            'speed_error_mps', 'throttle', 'steering', 'brake', 'enabled', 'reason',
            'visible_cones', 'command_age_s', 'status_age_s', 'camera_age_s',
            'steering_gain', 'steering_response', 'steering_limit', 'throttle_scale'])
        self.csv.writeheader()
        self.frame_handle = (self.output / 'frames.csv').open('x', newline='', encoding='utf-8')
        self.frame_csv = csv.DictWriter(self.frame_handle, fieldnames=[
            'index', 'file', 'elapsed_s', 'ros_stamp_s', 'width', 'height', 'encoding'])
        self.frame_csv.writeheader()
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.camera_topic, self.image_received,
                                 QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Odometry, '/fsds/testing_only/odom', self.odom_received, sensor_qos)
        self.create_subscription(String, '/car/command_debug', self.command_received, 20)
        self.create_subscription(String, '/car/status', self.status_received, 20)
        self.create_subscription(Log, '/rosout', self.rosout_received, sensor_qos)
        self.create_subscription(ParameterEvent, '/parameter_events',
                                 lambda msg: self.event('parameter_event', message_to_ordereddict(msg)), sensor_qos)
        self.create_subscription(ClockMessage, '/clock', self.clock_received, sensor_qos)
        try:
            from fs_msgs.msg import Track
            self.create_subscription(Track, '/fsds/testing_only/track', self.track_received,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        except ImportError:
            self.event('warning', {'message': 'fs_msgs unavailable; track snapshot disabled'})
        self.create_timer(1., self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.event('recording_started', {'utc': self.started_utc, 'camera_topic': self.camera_topic,
                   'schema_version': 2, 'timing': 'elapsed_s = monotonic receipt; ros_stamp_s = source'})
        self.flush()  # Readiness acknowledgement for the start/stop helper.

    def elapsed(self):
        return time.monotonic() - self.started

    def event(self, kind, data):
        self.events.write(json.dumps({'elapsed_s': self.elapsed(), 'kind': kind, 'data': data},
                                    ensure_ascii=False) + '\n')

    @staticmethod
    def stamp(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def receive_json(self, message, kind):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                raise ValueError('Expected object')
        except (ValueError, TypeError) as error:
            self.event('invalid_' + kind, {'error': str(error)})
            return
        setattr(self, kind, value)
        setattr(self, kind + '_time', self.elapsed())
        self.event(kind, value)

    def command_received(self, message):
        self.receive_json(message, 'command')

    def status_received(self, message):
        self.receive_json(message, 'status')

    def rosout_received(self, msg):
        self.event('rosout', message_to_ordereddict(msg))

    def clock_received(self, msg):
        current = self.stamp(msg.clock)
        if self.last_clock is not None and current < self.last_clock:
            self.event('clock_rewind', {'previous': self.last_clock, 'current': current})
            self.previous_position = None
            self.resets += 1
        self.last_clock = current

    def track_received(self, msg):
        value = message_to_ordereddict(msg)
        self.event('track', value)
        (self.output / 'track.json').write_text(json.dumps(value, indent=2), encoding='utf-8')

    def image_received(self, message):
        elapsed = self.elapsed()
        try:
            frame = decode_image(message)
        except ValueError as error:
            self.invalid_images += 1
            self.event('invalid_image', {'error': str(error), 'encoding': message.encoding})
            return
        stamp = self.stamp(message.header.stamp)
        if self.last_image_stamp is not None and stamp <= self.last_image_stamp:
            self.event('camera_nonincreasing_stamp', {'previous': self.last_image_stamp, 'current': stamp})
        if self.last_image_time is not None:
            self.max_camera_gap = max(self.max_camera_gap, elapsed - self.last_image_time)
        filename = 'frames/%08d.png' % len(self.frames)
        if not cv2.imwrite(str(self.output / filename), frame):
            raise OSError('Cannot save camera frame: ' + filename)
        row = {'index': len(self.frames), 'file': filename, 'elapsed_s': elapsed,
               'ros_stamp_s': stamp, 'width': message.width, 'height': message.height,
               'encoding': message.encoding}
        self.frames.append(row)
        self.frame_csv.writerow(row)
        self.frame_handle.flush()
        self.last_image_time, self.last_image_stamp = elapsed, stamp

    @staticmethod
    def age(now, then):
        return '' if then is None else max(0., now - then)

    def odom_received(self, message):
        now = self.elapsed()
        position, q = message.pose.pose.position, message.pose.pose.orientation
        velocity = message.twist.twist.linear
        speed = math.hypot(velocity.x, velocity.y)
        if not all(math.isfinite(v) for v in (position.x, position.y, position.z,
                                              speed, q.x, q.y, q.z, q.w)):
            self.invalid_odom += 1
            self.event('invalid_odom', {'message': 'Non-finite pose or speed'})
            return
        point = (position.x, position.y)
        if self.previous_position is not None:
            segment = math.dist(self.previous_position, point)
            if segment < 10.:
                self.distance += segment
            else:
                self.resets += 1
                self.event('pose_jump', {'distance_m': segment})
        self.previous_position = point
        self.maximum_speed = max(self.maximum_speed, speed)
        target = self.status.get('target_speed_mps')
        error = (target - speed if isinstance(target, (int, float)) and math.isfinite(target) else '')
        # Stale/disabled states must not pollute the speed-tracking metric.
        if (error != '' and self.status.get('enabled') and self.status_time is not None
                and now - self.status_time < .5):
            self.error_squared += error * error
            self.error_samples += 1
        row = {key: self.status.get(key, '') for key in (
            'target_speed_mps', 'enabled', 'reason', 'visible_cones',
            'steering_gain', 'steering_response', 'steering_limit', 'throttle_scale')}
        row.update({key: self.command.get(key, '') for key in ('throttle', 'steering', 'brake')})
        row.update({'elapsed_s': now, 'ros_stamp_s': self.stamp(message.header.stamp),
            'x_m': position.x, 'y_m': position.y, 'z_m': position.z,
            'yaw_rad': math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
            'vx_mps': velocity.x, 'vy_mps': velocity.y,
            'yaw_rate_radps': message.twist.twist.angular.z, 'speed_mps': speed,
            'speed_error_mps': error, 'command_age_s': self.age(now, self.command_time),
            'status_age_s': self.age(now, self.status_time),
            'camera_age_s': self.age(now, self.last_image_time)})
        self.csv.writerow(row)
        self.samples += 1
        self.odom_time = now

    def flush(self):
        self.csv_handle.flush()
        self.frame_handle.flush()
        self.events.flush()
        now = self.elapsed()
        health = {'state': 'recording', 'elapsed_s': now, 'camera_frames': len(self.frames),
                  'telemetry_samples': self.samples, 'camera_age_s': self.age(now, self.last_image_time),
                  'odom_age_s': self.age(now, self.odom_time), 'invalid_images': self.invalid_images}
        temporary = self.output / 'health.tmp'
        temporary.write_text(json.dumps(health), encoding='utf-8')
        temporary.replace(self.output / 'health.json')

    def close(self):
        if self.closed:
            return
        self.closed = True
        duration = self.elapsed()
        if self.last_image_time is not None:
            self.max_camera_gap = max(self.max_camera_gap, duration-self.last_image_time)
        self.flush()
        self.csv_handle.close()
        self.frame_handle.close()
        self.events.close()
        summary = {'schema_version': 2, 'started_utc': self.started_utc,
            'duration_s': duration, 'distance_m': self.distance,
            'maximum_speed_mps': self.maximum_speed, 'camera_frames': len(self.frames),
            'telemetry_samples': self.samples, 'camera_topic': self.camera_topic,
            'camera_received_fps': ((len(self.frames)-1) /
                (self.frames[-1]['elapsed_s']-self.frames[0]['elapsed_s'])
                if len(self.frames)>1 and self.frames[-1]['elapsed_s']>self.frames[0]['elapsed_s'] else 0.),
            'invalid_images': self.invalid_images, 'invalid_odom': self.invalid_odom,
            'pose_or_clock_resets': self.resets, 'maximum_camera_gap_s': self.max_camera_gap,
            'speed_rmse_mps': math.sqrt(self.error_squared / self.error_samples) if self.error_samples else None,
            'speed_rmse_samples': self.error_samples, 'video_created': False,
            'warnings': []}
        if not self.frames:
            summary['warnings'].append('No camera frames received: check topic, settings and bridge')
        if not self.samples:
            summary['warnings'].append('No odometry received')
        if self.max_camera_gap > 1.:
            summary['warnings'].append('Camera gaps exceed 1 s; MP4 holds the previous image during gaps')
        summary_path = self.output / 'summary.json'
        summary['state'] = 'finalizing'
        summary_path.write_text(json.dumps(summary, indent=2), encoding='utf-8')
        try:
            summary.update(assemble_video(self.output, self.frames, duration, self.fps))
            summary['state'] = 'complete'
        except Exception as error:
            summary['state'] = 'encoding_failed'
            summary['warnings'].append(str(error))
        summary_path.write_text(json.dumps(summary, indent=2), encoding='utf-8')
        (self.output / 'health.json').write_text(json.dumps({'state': summary['state']}), encoding='utf-8')
        report = ['# Отчёт о заезде', '', 'Начало (UTC): ' + self.started_utc,
            'Состояние: ' + summary['state'], '',
            '- Длительность: %.2f с' % duration,
            '- Пройдено: %.2f м' % self.distance,
            '- Максимальная скорость: %.2f м/с' % self.maximum_speed,
            '- Принято кадров: %d (%.2f кадр/с между первым и последним)' %
                (len(self.frames), summary['camera_received_fps']),
            '- Максимальная пауза камеры: %.2f с' % self.max_camera_gap,
            '- Отсчётов телеметрии: %d' % self.samples,
            '- RMSE скорости (свежий enabled status): %s м/с' % summary['speed_rmse_mps'],
            '', '## Предупреждения', '']
        report.extend('- '+warning for warning in summary['warnings'])
        if not summary['warnings']:
            report.append('Recorder не обнаружил перечисленных сбоев; это не оценка качества прохождения трассы.')
        report.extend(['', '## Как сопоставить видео и данные', '',
            'Время MP4 + video_start_elapsed_s из summary.json = elapsed_s в CSV.',
            'ROS source timestamps сохранены отдельно в frames.csv и telemetry.csv.',
            'Команды и status — последние полученные; проверяйте колонки *_age_s.',
            'frames/ содержит исходные кадры для повторной сборки видео.'])
        (self.output/'report.md').write_text('\n'.join(report)+'\n', encoding='utf-8')
        print('Recording complete: ' + str(summary_path), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = RunRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
