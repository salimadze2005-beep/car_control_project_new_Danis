"""Recorder writes playable camera output, telemetry and run summary."""
import json
from pathlib import Path
import tempfile

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from car_control_sim.recorder import RunRecorder


def main():
    with tempfile.TemporaryDirectory() as directory:
        rclpy.init(args=['--ros-args', '-p', 'output_dir:=%s' % directory,
                         '-p', 'video_fps:=10.0'])
        node = RunRecorder()
        try:
            node.status_received(String(data=json.dumps({
                'enabled': True, 'reason': 'running',
                'target_speed_mps': 2.5, 'visible_cones': 8})))
            node.command_received(String(data=json.dumps({
                'throttle': .2, 'steering': .1, 'brake': 0.})))
            image = Image()
            image.width, image.height = 64, 48
            image.encoding, image.step = 'bgr8', 64 * 3
            frame = np.zeros((48, 64, 3), dtype=np.uint8)
            frame[:, :, 1] = 180
            image.data = frame.tobytes()
            for _ in range(3):
                node.image_received(image)
            for x in (0., 1., 2.):
                odom = Odometry()
                odom.pose.pose.position.x = x
                odom.twist.twist.linear.x = 2.5
                node.odom_received(odom)
        finally:
            node.close()
            node.destroy_node()
            rclpy.shutdown()
        output = Path(directory)
        summary = json.loads((output / 'summary.json').read_text())
        assert summary['video_frames'] == 3
        assert summary['telemetry_samples'] == 3
        assert summary['distance_m'] == 2.0
        assert summary['maximum_speed_mps'] == 2.5
        assert (output / 'camera.mp4').stat().st_size > 0
        rows = (output / 'telemetry.csv').read_text().splitlines()
        assert len(rows) == 4 and 'visible_cones' in rows[0]
        print('FSDS camera video, telemetry CSV and summary JSON PASS')


if __name__ == '__main__':
    main()
