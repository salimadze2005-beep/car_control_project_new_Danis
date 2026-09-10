"""Hardware input adapter only: no serial, camera, TensorRT or ROS imports."""
from dataclasses import fields
from pathlib import Path
import sys

try:
    from car_control_core.core import Controller, Parameters, Command, fresh
except ModuleNotFoundError as error:
    if error.name != 'car_control_core':
        raise
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'shared'))
    from car_control_core.core import Controller, Parameters, Command, fresh


class HardwareAutopilot:
    def __init__(self, config):
        self.config = config
        values = {f.name: getattr(config, f.name) for f in fields(Parameters) if hasattr(config, f.name)}
        values['throttle'] = 1.0
        self.controller = Controller(Parameters(**values))
        self.was_enabled = False
        self.last_time = self.last_detection = None
        self.command = Command()

    def update(self, cones, enabled, detection_stamp, now):
        if enabled != self.was_enabled:
            self.controller.reset()
            self.last_time = self.last_detection = None
        self.was_enabled = enabled
        if not enabled or not fresh(detection_stamp, now, self.config.watchdog_timeout):
            if not self.controller.finished:
                self.controller.reset()
            self.command = Command()
            self.last_time = self.last_detection = None
            return self.command
        if detection_stamp == self.last_detection:
            return self.command
        names = {n: 'blue' for n in self.config.blue_cones}
        names.update({n: 'yellow' for n in self.config.yellow_cones})
        names.update({n: 'orange' for n in self.config.orange_cones})
        converted = [(c['pos_3d'][0], c['pos_3d'][1], names.get(c['name'], 'unknown')) for c in cones]
        dt = 0.02 if self.last_time is None else now - self.last_time
        self.command = self.controller.step(converted, dt)
        self.last_time, self.last_detection = now, detection_stamp
        return self.command
