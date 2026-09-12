import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'fsds_control_panel', ROOT / 'tools/fsds_control_panel.py')
PANEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PANEL)


class ControlPanelTests(unittest.TestCase):
    def test_tasklist_parser_and_duplicate_warning(self):
        output = ('"FSDS.exe","12676","Console","1","100 K"\n'
                  '"FSDS.exe","11212","Console","1","100 K"\n')
        self.assertEqual(PANEL.parse_tasklist_pids(output, 'FSDS.exe'),
                         [12676, 11212])
        warning = PANEL.duplicate_fsds_warning({
            'FSDS.exe': [12676, 11212], 'Blocks.exe': [15520, 13340]})
        self.assertIn('DUPLICATE FSDS', warning)
        self.assertIn('12676', warning)
        self.assertEqual(PANEL.duplicate_fsds_warning({
            'FSDS.exe': [11212], 'Blocks.exe': [13340]}), '')

    def test_wsl_invocation_is_one_literal_argument(self):
        command = PANEL.build_wsl_invocation(
            'Ubuntu-22.04', '/home/danis/car control', 'manual')
        self.assertEqual(command[:5], ['wsl.exe', '-d', 'Ubuntu-22.04', 'bash', '-lc'])
        self.assertIn("cd '/home/danis/car control'", command[5])
        self.assertTrue(command[5].endswith('python3 tools/fsds_mode.py manual'))

    def test_invalid_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            PANEL.build_wsl_invocation('Ubuntu-22.04', '/repo', 'turbo')

    def test_failed_mode_switch_remains_retryable(self):
        self.assertEqual(PANEL.mode_after_result('auto', 0), 'auto')
        self.assertIsNone(PANEL.mode_after_result('auto', 1))

    def test_speed_and_throttle_invocation(self):
        command = PANEL.build_speed_invocation(
            'Ubuntu-22.04', '/repo', 15.0, 0.25, 1.1, 0.65, 0.8)
        self.assertTrue(command[5].endswith(
            'python3 tools/fsds_speed.py 15.00 --throttle-scale 0.25 '
            '--steering-gain 1.10 --steering-response 0.65 '
            '--steering-limit 0.80'))
        for invalid in (0., 15.1, float('inf')):
            with self.assertRaises(ValueError):
                PANEL.build_speed_invocation('Ubuntu-22.04', '/repo', invalid, .2)
        with self.assertRaises(ValueError):
            PANEL.build_speed_invocation(
                'Ubuntu-22.04', '/repo', 2.5, .2, .8, 0.0, .7)

    def test_record_and_status_invocations(self):
        command = PANEL.build_record_invocation(
            'Ubuntu-22.04', '/repo', 'start')
        self.assertTrue(command[5].endswith('python3 tools/fsds_record.py start'))
        with self.assertRaises(ValueError):
            PANEL.build_record_invocation('Ubuntu-22.04', '/repo', 'erase')
        self.assertIn('ros2 topic echo /car/status --once',
                      PANEL.build_status_invocation('Ubuntu-22.04', '/repo')[5])

    def test_close_returns_keyboard_before_stopping_recorder(self):
        commands = PANEL.build_close_invocations('Ubuntu-22.04', '/repo')
        self.assertEqual(len(commands), 2)
        self.assertTrue(commands[0][5].endswith('python3 tools/fsds_mode.py manual'))
        self.assertTrue(commands[1][5].endswith('python3 tools/fsds_record.py stop'))


if __name__ == '__main__':
    unittest.main()
