import importlib.util
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'build_tensorrt_engine', ROOT/'tools/build_tensorrt_engine.py')
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)


class TensorRTBuilderTests(unittest.TestCase):
    def test_workspace_flag_tracks_tensorrt_version(self):
        self.assertEqual(BUILDER.workspace_argument('... --workspace=N ...', 512),
                         '--workspace=512')
        self.assertEqual(BUILDER.workspace_argument('... --memPoolSize=pool:size ...', 512),
                         '--memPoolSize=workspace:512')

    def test_static_model_command(self):
        command = BUILDER.build_command(
            Path('/trtexec'), Path('/model.onnx'), Path('/model.engine'),
            True, 1024, '--memPoolSize')
        self.assertEqual(command, [
            '/trtexec', '--onnx=/model.onnx', '--saveEngine=/model.engine',
            '--memPoolSize=workspace:1024', '--fp16'])

    def test_sha256(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'model.onnx'
            path.write_bytes(b'car-control')
            self.assertEqual(
                BUILDER.sha256(path),
                '3a19b4fd7371e2abcf7e7a753dc56ba2f501dfe061a1d4bb428ecb8ef118d693')


if __name__ == '__main__':
    unittest.main()
