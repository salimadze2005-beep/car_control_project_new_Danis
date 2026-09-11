"""Deterministic cone-sensor effects shared by ROS adapters and tests."""
from collections import deque
from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class SensorParameters:
    rate_hz: float = 0.0
    latency_s: float = 0.0
    dropout_probability: float = 0.0
    lateral_std_m: float = 0.0
    depth_std_m: float = 0.0
    depth_relative_std: float = 0.0
    camera_offset_x_m: float = 0.0
    camera_offset_z_m: float = 0.0
    min_depth_m: float = 0.1
    max_depth_m: float = 1000000.0
    max_per_color: int = 0
    seed: int = 0

    def __post_init__(self):
        numeric = (
            self.rate_hz, self.latency_s, self.dropout_probability,
            self.lateral_std_m, self.depth_std_m, self.depth_relative_std,
            self.camera_offset_x_m, self.camera_offset_z_m,
            self.min_depth_m, self.max_depth_m,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError('Sensor parameters must be finite')
        if self.rate_hz < 0 or self.latency_s < 0:
            raise ValueError('Sensor rate and latency cannot be negative')
        if not 0 <= self.dropout_probability <= 1:
            raise ValueError('dropout_probability must be in [0, 1]')
        if min(self.lateral_std_m, self.depth_std_m, self.depth_relative_std) < 0:
            raise ValueError('Sensor noise cannot be negative')
        if not 0 <= self.min_depth_m < self.max_depth_m:
            raise ValueError('Invalid sensor depth range')
        if int(self.max_per_color) != self.max_per_color or self.max_per_color < 0:
            raise ValueError('max_per_color must be a non-negative integer')


@dataclass(frozen=True)
class Observation:
    cones: tuple
    source_stamp: float
    received_stamp: float


class ConeSensor:
    """Rate-limit, offset, perturb and delay vehicle-frame cone observations.

    A zero-valued effect is a pass-through. Random effects are reproducible for
    a fixed seed so regression tests do not become flaky.
    """

    COLORS = ('blue', 'yellow', 'orange')

    def __init__(self, parameters):
        self.p = parameters
        self.random = random.Random(parameters.seed)
        self.reset()

    def reset(self):
        self.pending = deque()
        self.next_capture_stamp = None
        self.last_source_stamp = None

    def _noise(self, standard_deviation):
        return 0.0 if standard_deviation == 0 else self.random.gauss(0.0, standard_deviation)

    def capture(self, cones, source_stamp, received_stamp):
        if not all(math.isfinite(value) for value in (source_stamp, received_stamp)):
            self.reset()
            return False
        if self.last_source_stamp is not None and source_stamp < self.last_source_stamp:
            self.reset()
        self.last_source_stamp = source_stamp

        if self.p.rate_hz > 0:
            period = 1.0 / self.p.rate_hz
            if self.next_capture_stamp is None:
                self.next_capture_stamp = source_stamp
            if source_stamp + 1e-9 < self.next_capture_stamp:
                return False
            elapsed = max(0.0, source_stamp - self.next_capture_stamp)
            self.next_capture_stamp += (math.floor(elapsed / period) + 1) * period

        converted = []
        for x, z, color in cones:
            if color not in self.COLORS or not all(math.isfinite(value) for value in (x, z)):
                continue
            if self.p.dropout_probability and self.random.random() < self.p.dropout_probability:
                continue
            measured_z = z - self.p.camera_offset_z_m
            measured_z += self._noise(self.p.depth_std_m + abs(measured_z) * self.p.depth_relative_std)
            measured_x = x - self.p.camera_offset_x_m + self._noise(self.p.lateral_std_m)
            if self.p.min_depth_m < measured_z <= self.p.max_depth_m:
                converted.append((measured_x, measured_z, color))

        converted.sort(key=lambda cone: cone[1])
        if self.p.max_per_color:
            counts = {color: 0 for color in self.COLORS}
            limited = []
            for cone in converted:
                if counts[cone[2]] < self.p.max_per_color:
                    counts[cone[2]] += 1
                    limited.append(cone)
            converted = limited

        observation = Observation(tuple(converted), source_stamp, received_stamp)
        self.pending.append((received_stamp + self.p.latency_s, observation))
        while len(self.pending) > 64:
            self.pending.popleft()
        return True

    def ready(self, now):
        latest = None
        while self.pending and self.pending[0][0] <= now:
            _, latest = self.pending.popleft()
        return latest
