"""Wall-clock safety around the SAME controller; independent of ROS clock stalls."""
import math
from .core import Command, Controller, fresh, valid_command


class Session:
    def __init__(self, parameters, enabled=False, timeout=0.4, require_go=False):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive')
        self.core = Controller(parameters)
        self.timeout, self.require_go = timeout, require_go
        self.enabled = enabled
        self.sample = None
        self.go_time = None
        self.last_step_stamp = None
        self.last_output = Command()
        self.was_driving = False
        self.fault = False
        self.reason = 'waiting_for_data'

    def enable(self, enabled):
        self.enabled = bool(enabled)
        self.core.reset()
        self.sample = None
        self.last_step_stamp = None
        self.last_output = Command()
        self.was_driving = self.fault = False

    def observe(self, cones, source_stamp, received):
        if not math.isfinite(source_stamp) or not math.isfinite(received):
            self.sample = None
            return
        # Repeated frozen measurements must not keep the watchdog alive.
        if self.sample is not None and source_stamp <= self.sample[1]:
            if source_stamp < self.sample[1]:
                self.sample = None
                self.fault = True  # Clock reset: explicitly re-enable.
            return
        self.sample = (list(cones), source_stamp, received)

    def tick(self, now, source_now):
        sample = self.sample
        good = (sample is not None and math.isfinite(source_now) and source_now > 0
                and -0.1 <= source_now - sample[1] <= self.timeout
                and fresh(sample[2], now, self.timeout))
        go = not self.require_go or fresh(self.go_time, now, 4.)
        if not self.enabled:
            self.reason = 'disabled'
        elif self.fault:
            self.reason = 'connection_fault_reenable_required'
        elif not go:
            self.reason = 'no_go'
        elif not good:
            self.reason = 'stale_or_no_data'
        else:
            self.reason = 'running'
        if self.reason != 'running':
            if self.was_driving and (not good or not go):
                self.fault = True
            if not self.core.finished:
                self.core.reset()
            self.last_output = Command()
            return self.last_output
        if sample[1] != self.last_step_stamp:
            dt = 0.02 if self.last_step_stamp is None else sample[1] - self.last_step_stamp
            self.last_output = self.core.step(sample[0], dt)
            self.last_step_stamp = sample[1]
        if not valid_command(self.last_output):
            self.reason = 'invalid_command'
            self.last_output = Command()
        if self.last_output.brake:
            self.reason = 'finished' if self.core.finished else 'no_usable_cones'
        self.was_driving = self.was_driving or self.last_output.throttle > 0
        return self.last_output
