"""Pure tests: Vulkan workaround must apply before rendering the first frame."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    'fsds_launcher', Path(__file__).resolve().parents[1] / 'tools/launch_fsds_test.py')
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class LauncherTests(unittest.TestCase):
    def test_default_uses_startup_override(self):
        self.assertEqual(launcher.renderer_arguments(), [
            '-ini:Engine:[ConsoleVariables]:r.Vulkan.SubmitAfterEveryEndRenderPass=1'])

    def test_explicit_vulkan_is_also_safe(self):
        self.assertEqual(launcher.renderer_arguments('vulkan'),
                         ['-vulkan'] + launcher.renderer_arguments())

    def test_comparison_opt_out(self):
        self.assertEqual(launcher.renderer_arguments('auto', 'default'), [])

    def test_no_vulkan_setting_for_directx(self):
        self.assertEqual(launcher.renderer_arguments('d3d11'), ['-d3d11'])


if __name__ == '__main__':
    unittest.main()
