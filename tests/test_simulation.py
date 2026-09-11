import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'shared'), str(ROOT/'ros/car_control_ros/src'),
                str(ROOT/'Jetson Xavier')]
from car_control_core.core import Controller, Parameters, Command, Bicycle, circle_track, world_to_cones
from car_control_core.fsds import track_from_message, pose_from_message, vehicle_cones, map_command, validate_host, fill_command, speed_limited_command
from car_control_core.session import Session
from Code.Autopilot import HardwareAutopilot
from Code.Config_load import Config
import car_control_ros.core as ros1


def lane():
    return [(-0.75, 1., 'blue'), (0.75, 1., 'yellow')]


def odom(x=0., y=0., yaw=0.):
    return NS(header=NS(frame_id='fsds/map'), child_frame_id='fsds/FSCar',
              pose=NS(pose=NS(position=NS(x=x, y=y, z=0.),
                             orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)))))


class ContractTests(unittest.TestCase):
    def test_ros1_and_hardware_use_same_class(self):
        self.assertIs(ros1.Controller, Controller)
        self.assertIsInstance(HardwareAutopilot(Config()).controller, Controller)

    def test_all_cone_colors_and_units(self):
        msg = NS(track=[NS(location=NS(x=5., y=1., z=0.), color=i) for i in range(6)])
        result = track_from_message(msg)
        self.assertEqual([v[2] for v in result], ['blue','yellow','orange','orange'])
        self.assertEqual(result[0][:2], (5.,1.))  # No centimetre conversion a second time.

    def test_invalid_cone_coordinates(self):
        self.assertEqual(track_from_message(NS(track=[NS(location=NS(x=math.nan,y=0,z=0),color=0)])), [])

    def test_world_vehicle_translation_rotation(self):
        result = vehicle_cones([(9.,25.,'blue'),(11.,25.,'yellow')], odom(10.,20.,math.pi/2), 10.)
        self.assertAlmostEqual(result[0][0], -1.)
        self.assertAlmostEqual(result[0][1], 5.)
        self.assertAlmostEqual(result[1][0], 1.)

    def test_reject_frames(self):
        msg = odom()
        msg.header.frame_id = 'ned'
        with self.assertRaises(ValueError):
            pose_from_message(msg)

    def test_invalid_quaternion(self):
        msg = odom()
        msg.pose.pose.orientation.w = 0.
        with self.assertRaises(ValueError):
            pose_from_message(msg)
        msg.pose.pose.orientation.w = math.nan
        with self.assertRaises(ValueError):
            pose_from_message(msg)

    def test_command_saturation_and_sign(self):
        self.assertEqual(map_command(Command(2.,3.,-1.)), Command(1.,1.,0.))
        self.assertEqual(map_command(Command(0.2,-2.,0.)), Command(0.2,-1.,0.))
        self.assertEqual(map_command(Command(-1.,0.,0.)), Command(0.,0.,0.))

    def test_brake_overrides_throttle(self):
        self.assertEqual(map_command(Command(1.,0.2,0.4)), Command(0.,0.2,0.4))
        self.assertEqual(map_command(Command(math.nan,0.,0.)), Command())
        self.assertEqual(map_command(Command(0.,math.inf,0.)), Command())

    def test_fsds_speed_governor(self):
        command = Command(0.2, -0.3, 0.)
        self.assertEqual(speed_limited_command(command, 1.9, 2., 0.25), command)
        self.assertEqual(speed_limited_command(command, 2., 2., 0.25), Command(0., -0.3, 0.25))

    def test_message_stamp(self):
        msg = fill_command(NS(header=NS()), Command(0.1,-0.5,0.), 'stamp')
        self.assertEqual(msg.header.stamp, 'stamp')
        self.assertEqual(msg.steering, -0.5)

    def test_local_remote_hosts(self):
        for host in ('localhost','192.168.1.10','fsds.example.org','::1'):
            self.assertEqual(validate_host(host), host)
        for bad in ('', 'http://localhost', 'host:41451', 'a b', 'x;echo', '../host', '-bad', 'a..b'):
            with self.assertRaises(ValueError):
                validate_host(bad)

    def test_deterministic_two_laps(self):
        states = []
        for _ in range(2):
            car, control, track = Bicycle(), Controller(Parameters()), circle_track()
            worst = 0.
            for _ in range(12000):
                car.step(control.step(world_to_cones(track, car.x, car.y, car.yaw, 4.), 0.02), 0.02)
                worst = max(worst, abs(math.hypot(car.x,car.y-8.)-8.))
            self.assertGreater(car.yaw, 4*math.pi)
            self.assertLess(worst, 0.5)
            states.append((car.x,car.y,car.yaw))
        self.assertEqual(*states)

    def test_hardware_adapter_matches_core(self):
        hw = HardwareAutopilot(Config())
        core = Controller(Parameters(throttle=1.))
        cones = [{'pos_3d': (x,z), 'name': c} for x,z,c in lane()]
        self.assertEqual(hw.update(cones, True, 10.,10.), core.step(lane(),0.02))
        self.assertEqual(hw.update(cones, True, 10.,11.), Command())
        self.assertEqual(hw.update(cones, False, 11.,11.), Command())


class SafetyTests(unittest.TestCase):
    def test_disabled_start_and_explicit_enable(self):
        s = Session(Parameters())
        s.observe(lane(), 10.,1.)
        self.assertEqual(s.tick(1.,10.),Command())
        s.enable(True)
        self.assertEqual(s.tick(1.,10.),Command())
        s.observe(lane(),10.02,1.02)
        self.assertGreater(s.tick(1.02,10.02).throttle,0.)

    def test_stale_source_not_refreshed_by_receipt(self):
        s = Session(Parameters(),enabled=True)
        s.observe(lane(),10.,1.)
        self.assertEqual(s.tick(1.,11.),Command())

    def test_frozen_source_and_reconnection_latch(self):
        s = Session(Parameters(),enabled=True)
        s.observe(lane(),10.,1.)
        self.assertGreater(s.tick(1.,10.).throttle,0.)
        s.observe(lane(),10.,1.8)
        self.assertEqual(s.tick(1.8,10.),Command())
        s.observe(lane(),11.,2.)
        self.assertEqual(s.tick(2.,11.),Command())
        s.enable(True)
        s.observe(lane(),11.1,2.1)
        self.assertGreater(s.tick(2.1,11.1).throttle,0.)

    def test_empty_cones_and_invalid_command(self):
        s = Session(Parameters(),enabled=True)
        s.observe([],10.,1.)
        self.assertEqual(s.tick(1.,10.),Command())
        s.last_output=Command(2.,0.,0.)
        self.assertEqual(s.tick(1.,10.),Command())
        self.assertEqual(s.reason, 'invalid_command')
        self.assertTrue(s.fault)

    def test_go_heartbeat_expiration(self):
        s = Session(Parameters(),enabled=True,require_go=True)
        s.observe(lane(),10.,1.)
        self.assertEqual(s.tick(1.,10.),Command())
        s.go_time = 1.
        self.assertGreater(s.tick(1.,10.).throttle,0.)
        s.observe(lane(),15.,6.)
        self.assertEqual(s.tick(6.,15.),Command())

    def test_clock_rewind_and_future(self):
        s = Session(Parameters(),enabled=True)
        s.observe(lane(),10.,1.)
        s.tick(1.,10.)
        s.observe(lane(),2.,2.)
        self.assertEqual(s.tick(2.,2.),Command())
        s.enable(True)
        s.observe(lane(),10.,2.)
        self.assertEqual(s.tick(2.,2.),Command())


if __name__ == '__main__':
    unittest.main()
