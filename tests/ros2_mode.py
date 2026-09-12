"""Mode helper clears a simulated connection-fault latch before AUTO."""
import json
from pathlib import Path
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.fsds_mode import set_mode


class Harness(Node):
    def __init__(self):
        super().__init__('fsds_mode_harness')
        self.enabled = False
        self.manual = True
        self.enable_attempts = 0
        self.publisher = self.create_publisher(String, '/car/status', 10)
        self.create_service(SetBool, '/car/enable', self.enable)
        self.create_service(SetBool, '/fsds/manual_mode', self.set_manual)
        self.create_timer(0.02, self.publish_status)

    def enable(self, request, response):
        self.enabled = request.data
        if request.data:
            self.enable_attempts += 1
        response.success = True
        response.message = 'ok'
        return response

    def set_manual(self, request, response):
        self.manual = request.data
        response.success = True
        response.message = 'ok'
        return response

    def publish_status(self):
        if not self.enabled:
            reason = 'disabled'
        elif self.enable_attempts == 1:
            reason = 'connection_fault_reenable_required'
        else:
            reason = 'running'
        self.publisher.publish(String(data=json.dumps({
            'backend': 'fsds', 'enabled': self.enabled, 'reason': reason,
            'visible_cones': 8})))


def main():
    rclpy.init()
    harness = Harness()
    executor = MultiThreadedExecutor()
    executor.add_node(harness)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = Node('fsds_mode_test_client')
    try:
        assert set_mode(client, 'auto').startswith('AUTO')
        assert harness.enable_attempts == 2
        assert harness.enabled and not harness.manual
        assert set_mode(client, 'manual').startswith('MANUAL')
        assert not harness.enabled and harness.manual
        print('Mode retry, fault-latch recovery and MANUAL handoff PASS')
    finally:
        executor.shutdown()
        client.destroy_node()
        harness.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.)


if __name__ == '__main__':
    main()
