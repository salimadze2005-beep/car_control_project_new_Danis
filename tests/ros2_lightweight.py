"""ROS2 lightweight graph with no fs_msgs, camera, GPU or hardware dependency."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from car_control_sim.node import ControllerNode


def main():
    rclpy.init(args=['--ros-args', '-p', 'backend:=lightweight', '-p', 'auto_start:=true'])
    controller, observer = ControllerNode(), Node('lightweight_test')
    commands, poses = [], []
    observer.create_subscription(String, '/car/command_debug', lambda m: commands.append(json.loads(m.data)), 10)
    observer.create_subscription(Odometry, '/car/sim/odom', poses.append, 10)
    client = observer.create_client(SetBool, '/car/enable')
    ex = SingleThreadedExecutor()
    ex.add_node(controller)
    ex.add_node(observer)

    def pump(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            ex.spin_once(timeout_sec=0.01)

    try:
        pump(2.)
        assert len(commands) > 20 and len(poses) > 20
        assert commands[-1]['throttle'] > 0 and poses[-1].pose.pose.position.x > 0.1
        future = client.call_async(SetBool.Request(data=False))
        pump(0.6)
        assert future.done() and future.result().success
        assert commands[-1]['throttle'] == 0 and commands[-1]['brake'] == 1
        assert abs(poses[-1].twist.twist.linear.x) < 0.001
        print('ROS2 lightweight: DDS, closed-loop movement and disable/brake PASS')
    finally:
        ex.shutdown()
        controller.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
