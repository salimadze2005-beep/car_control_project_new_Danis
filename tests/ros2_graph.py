"""Real DDS messages and service calls with a fake FSDS publisher; NOT Unreal E2E."""
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from fs_msgs.msg import Track, Cone, GoSignal, ControlCommand
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from car_control_sim.node import ControllerNode


def main():
    rclpy.init(args=['--ros-args', '-p', 'backend:=fsds', '-p', 'use_sim_time:=true'])
    fake = Node('fsds_fixture')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    track_pub = fake.create_publisher(Track, '/fsds/testing_only/track', latched)
    clock_pub = fake.create_publisher(Clock, '/clock', 10)
    odom_pub = fake.create_publisher(Odometry, '/fsds/testing_only/odom', 10)
    go_pub = fake.create_publisher(GoSignal, '/fsds/signal/go', 1)
    received = []
    fake.create_subscription(ControlCommand, '/fsds/control_command', received.append, 10)
    track = Track()
    for y,color in [(0.75,Cone.BLUE),(-0.75,Cone.YELLOW)]:
        cone = Cone()
        cone.location.x,cone.location.y,cone.color=1.,y,color
        track.track.append(cone)
    track_pub.publish(track)  # Must reach a subscriber created later.
    controller = ControllerNode()
    ex = SingleThreadedExecutor()
    ex.add_node(fake)
    ex.add_node(controller)
    client = fake.create_client(SetBool, '/car/enable')
    start = time.monotonic()
    frozen = None

    def pump(seconds, odometry=True, freeze=False):
        nonlocal frozen
        until = time.monotonic()+seconds
        while time.monotonic()<until:
            stamp = int((1000.+time.monotonic()-start)*1e9)
            if freeze:
                stamp = frozen if frozen is not None else stamp
                frozen = stamp
            clock = Clock()
            clock.clock.sec, clock.clock.nanosec = divmod(stamp,10**9)
            clock_pub.publish(clock)
            if odometry:
                msg = Odometry()
                msg.header.frame_id,msg.child_frame_id='fsds/map','fsds/FSCar'
                msg.header.stamp=clock.clock
                msg.pose.pose.orientation.w=1.
                odom_pub.publish(msg)
            go_pub.publish(GoSignal())
            ex.spin_once(timeout_sec=0.005)
            # Drain ready callbacks, then allow wall time to advance.
            for _ in range(8):
                ex.spin_once(timeout_sec=0.)
            time.sleep(0.01)

    def enable(value):
        future = client.call_async(SetBool.Request(data=value))
        pump(0.3)
        assert future.done() and future.result().success

    try:
        pump(1.)
        assert controller.track, 'Transient-local upstream track missing'
        assert received and received[-1].brake==1., 'Disabled controller did not brake'
        enable(True)
        pump(0.4)
        assert received[-1].throttle>0 and received[-1].brake==0
        enable(False)
        assert received[-1].throttle==0 and received[-1].brake==1
        enable(True)
        pump(0.8,odometry=False)
        assert received[-1].throttle==0 and received[-1].brake==1
        pump(0.3)
        assert received[-1].brake==1, 'Reconnect must require enable'
        enable(True)
        pump(0.8,freeze=True)
        assert received[-1].throttle==0 and received[-1].brake==1, 'Frozen /clock must not defeat watchdog'
        print('ROS2 DDS: latched track, enable/disable, motion, sensor loss, reconnect latch, frozen clock PASS')
    finally:
        controller.stop()
        ex.shutdown()
        controller.destroy_node()
        fake.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
