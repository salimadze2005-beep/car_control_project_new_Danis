"""Live parameter updates affect FSDS output; rejected updates and STOP stay safe."""
import rclpy
from rclpy.parameter import Parameter
from car_control_sim.node import ControllerNode
from car_control_core.core import Command
from car_control_core.fsds import LongitudinalController


def main():
    rclpy.init(args=['--ros-args', '-p', 'backend:=fsds',
                     '-p', 'fsds_longitudinal_mode:=pi'])
    node = ControllerNode()
    try:
        def set_speed(value):
            return node.set_parameters_atomically([
                Parameter('fsds_max_speed_mps', value=value)])
        assert set_speed(15.0).successful
        node.tick()
        assert node.fsds_max_speed_mps == 15.0
        assert node.fsds_longitudinal_mode == 'pi'
        assert not node.session.enabled
        moving = Command(1., 0., 0.)
        regulator = LongitudinalController()
        assert regulator.step(moving, 2.5, node.fsds_max_speed_mps, .2, .05).throttle > 0
        assert set_speed(1.5).successful
        node.tick()
        assert regulator.step(moving, 2.5, node.fsds_max_speed_mps, .2, .05).brake > 0
        for bad in (float('nan'), float('inf'), -1., 0., 15.1):
            assert not set_speed(bad).successful
            assert node.get_parameter('fsds_max_speed_mps').value == 1.5
        for speed in (0., 2.5, 2.6, 16.):
            assert regulator.step(Command(), speed, 2.5, 1., .05).brake == 1.
        print('Runtime speed updates, validation, disabled state and brake priority PASS')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
