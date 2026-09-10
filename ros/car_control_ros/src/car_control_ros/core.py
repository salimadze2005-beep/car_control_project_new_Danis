"""Compatibility import for ROS1 callers; mathematics lives in shared/."""
from pathlib import Path
import sys

try:
    from car_control_core.core import *
except ModuleNotFoundError as error:
    if error.name != 'car_control_core':
        raise
    sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'shared'))
    from car_control_core.core import *
