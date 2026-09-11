"""Fetch only pinned upstream sources; never download/copy Unreal scenes into this repo."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def git(*args, cwd=None):
    return subprocess.check_output(['git', *map(str, args)], cwd=cwd, text=True).strip()


def prepare(destination):
    lock = json.loads((ROOT / 'simulation/fsds.lock.json').read_text())
    destination = Path(destination).expanduser().resolve()
    if destination == ROOT or ROOT in destination.parents:
        raise ValueError('FSDS must be a separate checkout OUTSIDE the car repository')
    if destination.exists():
        if not (destination / '.git').is_dir():
            raise ValueError('Existing destination is not a Git checkout')
        if git('rev-parse', 'HEAD', cwd=destination) != lock['commit']:
            raise ValueError('Existing checkout is not the pinned FSDS revision; choose an empty destination')
    else:
        git('clone', '--filter=blob:none', '--no-checkout', '--depth', '1',
            '--branch', lock['tag'], lock['repository'], destination)
        git('sparse-checkout', 'set', 'AirSim', 'ros2', cwd=destination)
        git('checkout', lock['commit'], cwd=destination)
    if git('rev-parse', 'HEAD', cwd=destination) != lock['commit']:
        raise ValueError('Upstream tag no longer points to pinned commit')
    git('submodule', 'update', '--init', 'ros2/src/fs_msgs', 'AirSim/external/rpclib', cwd=destination)
    for path, key in [('ros2/src/fs_msgs', 'fs_msgs_ros2_commit'), ('AirSim/external/rpclib', 'rpclib_commit')]:
        if git('rev-parse', 'HEAD', cwd=destination/path) != lock[key]:
            raise ValueError('Unexpected submodule revision: ' + path)
    patch = ROOT / 'simulation' / lock['patch']
    # Upstream blobs mix CRLF/LF. Context matching must work on Linux and Windows.
    apply_options = ['--unidiff-zero', '--ignore-space-change']
    check = subprocess.run(
        ['git', 'apply', *apply_options, '--reverse', '--check', str(patch)],
        cwd=destination, capture_output=True)
    if check.returncode:
        git('apply', *apply_options, '--check', patch, cwd=destination)
        git('apply', *apply_options, patch, cwd=destination)
    print('Pinned FSDS ROS2 sources ready:', destination)
    print('Build on Ubuntu 22.04 / Humble; see docs/SIMULATION_SETUP.md.')
    return destination


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--destination', required=True)
    args = parser.parse_args()
    prepare(args.destination)
