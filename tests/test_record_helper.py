import importlib.util
import json
import os
from pathlib import Path
import tempfile
from unittest import TestCase, mock
from unittest import SkipTest

if os.name == 'nt':
    raise SkipTest('Recorder process lifecycle uses Linux /proc and runs in WSL')

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('record_helper', ROOT/'tools/fsds_record.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


class RecorderHelperTests(TestCase):
    def test_start_waits_for_recorder_readiness(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'ros2/car_control_sim/config').mkdir(parents=True)
            (root/'simulation').mkdir()
            (root/'simulation/fsds-settings.json').write_text('{}')
            sleeps = []
            def loading_imports(seconds):
                sleeps.append(seconds)
                if len(sleeps) == 3:
                    output = next((root/'recordings').iterdir())
                    (output/'health.json').write_text(json.dumps({'state':'recording'}))
            process = mock.Mock(pid=1234)
            process.poll.return_value = None
            with mock.patch.object(helper, 'ROOT', root), \
                 mock.patch.object(helper, 'STATE', root/'state.json'), \
                 mock.patch.object(helper.subprocess, 'run', return_value=mock.Mock(stdout='test')), \
                 mock.patch.object(helper.subprocess, 'Popen', return_value=process), \
                 mock.patch.object(helper.time, 'sleep', side_effect=loading_imports):
                self.assertEqual(helper.start(), 0)
                self.assertEqual(len(sleeps), 3)
                self.assertEqual(json.loads((root/'state.json').read_text())['pid'], 1234)
