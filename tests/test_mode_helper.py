import importlib.util
from pathlib import Path
import sys
import types
import unittest


ROOT = Path(__file__).resolve().parents[1]
RCLPY = types.ModuleType('rclpy')
RCLPY_NODE = types.ModuleType('rclpy.node')
RCLPY_NODE.Node = object
STD_MSGS = types.ModuleType('std_msgs')
STD_MSGS_MSG = types.ModuleType('std_msgs.msg')
STD_MSGS_MSG.String = object
STD_SRVS = types.ModuleType('std_srvs')
STD_SRVS_SRV = types.ModuleType('std_srvs.srv')
STD_SRVS_SRV.SetBool = object
sys.modules.update({
    'rclpy': RCLPY, 'rclpy.node': RCLPY_NODE,
    'std_msgs': STD_MSGS, 'std_msgs.msg': STD_MSGS_MSG,
    'std_srvs': STD_SRVS, 'std_srvs.srv': STD_SRVS_SRV,
})
SPEC = importlib.util.spec_from_file_location(
    'fsds_mode', ROOT / 'tools/fsds_mode.py')
MODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODE)


class ModeHelperTests(unittest.TestCase):
    def test_enable_requires_fresh_visible_cones(self):
        ready = {'backend': 'fsds', 'enabled': False, 'reason': 'disabled',
                 'visible_cones': 8}
        self.assertTrue(MODE.ready_for_enable(ready))
        for update in ({'visible_cones': 0}, {'enabled': True},
                       {'reason': 'connection_fault_reenable_required'},
                       {'backend': 'lightweight'}):
            status = dict(ready)
            status.update(update)
            self.assertFalse(MODE.ready_for_enable(status))


if __name__ == '__main__':
    unittest.main()
