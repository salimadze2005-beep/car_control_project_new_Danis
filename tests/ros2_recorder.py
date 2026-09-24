"""Exercise real ROS messages, recoverable images, metrics and shutdown."""
import csv
import json
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from car_control_sim.recorder import RunRecorder


def main():
    with tempfile.TemporaryDirectory() as directory:
        rclpy.init(args=['--ros-args', '-p', 'output_dir:=%s' % directory, '-p', 'video_fps:=10.0'])
        node = RunRecorder()
        try:
            node.status_received(String(data=json.dumps({'enabled': True, 'reason': 'running',
                'target_speed_mps': 2.5, 'visible_cones': 8})))
            node.command_received(String(data=json.dumps({'throttle': .2, 'steering': .1, 'brake': 0.})))
            image = Image()
            image.width, image.height = 64, 48
            image.encoding, image.step = 'rgb8', 64*3
            image.data = np.full((48,64,3), 100, np.uint8).tobytes()
            for i in range(3):
                image.header.stamp.sec = i+1
                node.image_received(image)
                odom = Odometry()
                odom.pose.pose.position.x = float(i)
                odom.pose.pose.orientation.w = 1.
                odom.twist.twist.linear.x = 2.5
                node.odom_received(odom)
                time.sleep(.11)
            image.encoding = 'invalid'
            node.image_received(image)
            node.status_received(String(data='not json'))
        finally:
            node.close()
            node.close()  # idempotent finalization
            node.destroy_node()
            rclpy.shutdown()
        output = Path(directory)
        summary = json.loads((output/'summary.json').read_text())
        assert summary['camera_frames'] == 3 and summary['invalid_images'] == 1
        assert summary['telemetry_samples'] == 3 and summary['distance_m'] == 2.
        assert summary['maximum_speed_mps'] == 2.5 and summary['speed_rmse_mps'] == 0.
        assert summary['state'] == 'complete' and summary['video_created']
        capture = cv2.VideoCapture(str(output/'camera.mp4'))
        assert capture.isOpened() and capture.read()[0]
        capture.release()
        with (output/'telemetry.csv').open() as handle:
            rows = list(csv.DictReader(handle))
        assert len(rows) == 3 and float(rows[0]['command_age_s']) >= 0
        assert len(list((output/'frames').glob('*.png'))) == 3
        events = [json.loads(line) for line in (output/'events.jsonl').read_text().splitlines()]
        assert any(e['kind']=='invalid_image' for e in events)
        assert any(e['kind']=='invalid_status' for e in events)
        print('Recorder: playable timed MP4, PNG recovery, events, telemetry, metrics PASS')


if __name__ == '__main__':
    main()
