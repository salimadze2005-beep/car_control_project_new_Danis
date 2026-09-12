#!/usr/bin/env python3
"""Start/stop the independent ROS2 FSDS camera and telemetry recorder."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time


ROOT = Path(__file__).resolve().parents[1]
STATE = Path('/tmp/car_control_fsds_recorder_%d.json' % os.getuid())


def load_state():
    try:
        return json.loads(STATE.read_text(encoding='utf-8'))
    except (FileNotFoundError, ValueError):
        return None


def recorder_alive(state):
    if not state:
        return False
    pid = int(state.get('pid', -1))
    try:
        command = Path('/proc/%d/cmdline' % pid).read_bytes().replace(b'\0', b' ')
    except (FileNotFoundError, PermissionError, ValueError):
        return False
    return b'car_control_sim' in command and b'recorder' in command


def start():
    state = load_state()
    if recorder_alive(state):
        print('Recording already active:', state['output'])
        return 0
    output = ROOT / 'recordings' / datetime.now().strftime('%Y%m%d_%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    log_path = output / 'recorder.log'
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'car_control_sim', 'recorder', '--ros-args',
            '-p', 'output_dir:=%s' % output], stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True)
    state = {'pid': process.pid, 'output': str(output), 'log': str(log_path)}
    STATE.write_text(json.dumps(state), encoding='utf-8')
    time.sleep(0.6)
    if process.poll() is not None:
        STATE.unlink(missing_ok=True)
        tail = log_path.read_text(encoding='utf-8', errors='replace')[-1000:]
        print('Recorder failed to start:', tail)
        return 1
    print('Recording started:', output)
    return 0


def stop():
    state = load_state()
    if not recorder_alive(state):
        STATE.unlink(missing_ok=True)
        print('Recording is not active')
        return 0
    pid = int(state['pid'])
    os.killpg(pid, signal.SIGINT)
    for _ in range(50):
        if not recorder_alive(state):
            break
        time.sleep(0.1)
    if recorder_alive(state):
        os.killpg(pid, signal.SIGTERM)
    STATE.unlink(missing_ok=True)
    print('Recording stopped:', state['output'])
    return 0


def status():
    state = load_state()
    if recorder_alive(state):
        print('Recording active:', state['output'])
    else:
        print('Recording is not active')
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('start', 'stop', 'status'))
    args = parser.parse_args()
    return {'start': start, 'stop': stop, 'status': status}[args.action]()


if __name__ == '__main__':
    raise SystemExit(main())
