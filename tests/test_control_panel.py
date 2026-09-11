import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'fsds_control_panel', ROOT / 'tools/fsds_control_panel.py')
PANEL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PANEL)


class ControlPanelTests(unittest.TestCase):
    def test_wsl_invocation_is_one_literal_argument(self):
        command = PANEL.build_wsl_invocation(
            'Ubuntu-22.04', '/home/danis/car control', 'manual')
        self.assertEqual(command[:5], ['wsl.exe', '-d', 'Ubuntu-22.04', 'bash', '-lc'])
        self.assertIn("cd '/home/danis/car control'", command[5])
        self.assertTrue(command[5].endswith('python3 tools/fsds_mode.py manual'))

    def test_invalid_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            PANEL.build_wsl_invocation('Ubuntu-22.04', '/repo', 'turbo')


if __name__ == '__main__':
    unittest.main()
