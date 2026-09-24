"""Camera colour/stride validation and nonuniform frame timing regression."""
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('recording', ROOT/'ros2/car_control_sim/car_control_sim/recording.py')
recording = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(recording)
except ImportError:
    recording = None


@unittest.skipIf(recording is None, 'OpenCV/numpy required (ROS environment has both)')
class RecordingTests(unittest.TestCase):
    def test_rgb_padded_rows(self):
        message = SimpleNamespace(encoding='rgb8', width=1, height=2, step=4,
                                  data=bytes([255,0,0,99,0,255,0,99]))
        image = recording.decode_image(message)
        self.assertEqual(image.tolist(), [[[0,0,255]], [[0,255,0]]])
        message.data = message.data[:-1]
        with self.assertRaises(ValueError):
            recording.decode_image(message)

    def test_mono_and_rgba(self):
        for encoding, data, expected in [('mono8', [70], [70,70,70]),
                                         ('rgba8', [255,0,0,10], [0,0,255])]:
            message = SimpleNamespace(encoding=encoding, width=1, height=1,
                                      step=len(data), data=bytes(data))
            self.assertEqual(recording.decode_image(message)[0,0].tolist(), expected)

    def test_irregular_frames_preserve_wall_time_and_gaps(self):
        cv2, np = recording.cv2, recording.np
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            frames = []
            for i, (timestamp, color) in enumerate([(2., (0,0,255)), (2.5, (0,255,0)), (4., (255,0,0))]):
                frame = np.zeros((48,64,3), np.uint8)
                frame[:] = color
                filename = '%d.png' % i
                self.assertTrue(cv2.imwrite(str(output/filename), frame))
                frames.append({'elapsed_s': timestamp, 'file': filename})
            result = recording.assemble_video(output, frames, 5., 10.)
            self.assertEqual(result['video_frames'], 30)
            self.assertEqual(result['video_duration_s'], 3.)
            self.assertEqual(result['video_start_elapsed_s'], 2.)
            capture = cv2.VideoCapture(str(output/'camera.mp4'))
            colors = []
            while True:
                ok, image = capture.read()
                if not ok:
                    break
                colors.append(int(image.mean(axis=(0,1)).argmax()))
            capture.release()
            self.assertEqual(colors, [2]*5 + [1]*15 + [0]*10)


if __name__ == '__main__':
    unittest.main()
