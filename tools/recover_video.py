#!/usr/bin/env python3
"""Rebuild a missing/failed MP4 from preserved PNGs and frames.csv."""
import argparse
import csv
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('recording', ROOT/'ros2/car_control_sim/car_control_sim/recording.py')
recording = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recording)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--fps', type=float, default=15.)
    args = parser.parse_args()
    if not 1 <= args.fps <= 120:
        parser.error('FPS must be within 1..120')
    if (args.run/'camera.mp4').exists():
        parser.error('camera.mp4 already exists; move it aside before rebuilding')
    with (args.run/'frames.csv').open(newline='') as handle:
        frames = list(csv.DictReader(handle))
    if not frames:
        parser.error('No source frames')
    for frame in frames:
        frame['elapsed_s'] = float(frame['elapsed_s'])
        if not (args.run/frame['file']).resolve().is_relative_to(args.run.resolve()):
            parser.error('Frame path escapes recording directory')
    end = frames[-1]['elapsed_s'] + 1/args.fps
    summary_path = args.run/'summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    end = max(end, summary.get('duration_s', end))
    result = recording.assemble_video(args.run, frames, end, args.fps)
    summary.update(result)
    summary['state'] = 'recovered'
    summary_path.write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
