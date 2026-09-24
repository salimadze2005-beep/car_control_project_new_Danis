import csv
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('tracks', ROOT/'tools/generate_test_tracks.py')
tracks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracks)


class TrackTests(unittest.TestCase):
    def test_fsds_loader_contract_and_repeatability(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            tracks.generate(a)
            tracks.generate(b)
            for name in ('figure_eight', 'slalom', 'turns', 'test_ground'):
                path = Path(a)/(name+'.csv')
                self.assertEqual(path.read_bytes(), (Path(b)/path.name).read_bytes())
                with path.open() as handle:
                    rows = list(csv.reader(handle))
                self.assertGreater(len(rows), 60)
                self.assertEqual(sum(row[0]=='big_orange' for row in rows), 2)
                for row in rows:
                    self.assertEqual(len(row), 7)
                    self.assertIn(row[0], ('blue','yellow','big_orange'))
                    self.assertTrue(all(math.isfinite(float(v)) for v in row[1:]))
                gate = [row for row in rows if row[0]=='big_orange']
                self.assertAlmostEqual(math.dist([float(v) for v in gate[0][1:3]],
                                                [float(v) for v in gate[1][1:3]]), 4.)
                heading = math.atan2(-float(gate[0][2])+float(gate[1][2]),
                                     float(gate[0][1])-float(gate[1][1]))-math.pi/2
                self.assertAlmostEqual(heading, 0.)

    def test_spacing_and_initial_direction(self):
        for name, (points, closed) in tracks.centerlines().items():
            self.assertAlmostEqual(points[0][0], 0.)
            self.assertAlmostEqual(points[0][1], 0.)
            self.assertGreater(points[1][0], 0.)
            self.assertAlmostEqual(points[1][1], 0.)
            self.assertTrue(all(math.dist(a,b) <= 2.001 for a,b in zip(points,points[1:])))


if __name__ == '__main__':
    unittest.main()
