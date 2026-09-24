"""FSDS camera settings are Z-up, unlike upstream AirSim's NED convention."""
import json
from pathlib import Path
import unittest


class CameraConfigTests(unittest.TestCase):
    def test_front_camera_is_above_vehicle_origin(self):
        root = Path(__file__).resolve().parents[1]
        settings = json.loads((root/'simulation/fsds-settings.json').read_text())
        camera = settings['Vehicles']['FSCar']['Cameras']['front']
        # FSDS 2.2.0 CarPawnSimApi::createCamerasFromSettings uses fromLocalEnu:
        # X forward, Y left, Z up. A negative Z puts the camera under the road.
        self.assertGreater(camera['Z'], 0., 'FSDS front camera must be above the car origin')
        self.assertGreater(camera['X'], 0., 'Front camera must face the driving corridor')


if __name__ == '__main__':
    unittest.main()
