#!/usr/bin/env python3
"""Start/stop the independent ROS2 FSDS camera and telemetry recorder."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import shutil
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
    output = ROOT / 'recordings' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    metadata = {'schema_version': 2, 'repository': str(ROOT),
                'created_local': datetime.now().astimezone().isoformat(),
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0')}
    for key, command in [('git_commit', ['git', 'rev-parse', 'HEAD']),
                         ('git_status', ['git', 'status', '--short']),
                         ('git_diff', ['git', 'diff', '--no-ext-diff'])]:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=10)
        if key == 'git_diff':
            (output / 'source_changes.patch').write_text(result.stdout, encoding='utf-8')
        else:
            metadata[key] = result.stdout.strip()
    (output / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    configs = output / 'configs'
    configs.mkdir()
    for source in sorted((ROOT / 'ros2/car_control_sim/config').glob('*.json')):
        shutil.copy2(source, configs / source.name)
    shutil.copy2(ROOT / 'simulation/fsds-settings.json', configs / 'fsds-settings.json')
    log_path = output / 'recorder.log'
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen([
            'ros2', 'run', 'car_control_sim', 'recorder', '--ros-args',
            '-p', 'output_dir:=%s' % output], stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True)
    state = {'pid': process.pid, 'output': str(output), 'log': str(log_path)}
    STATE.write_text(json.dumps(state), encoding='utf-8')
    # A live `ros2 run` wrapper is not proof that OpenCV imports/node startup
    # finished. Do not acknowledge START before the recorder opens its logs.
    for _ in range(100):
        if process.poll() is not None or (output / 'health.json').exists():
            break
        time.sleep(0.1)
    if process.poll() is not None:
        STATE.unlink(missing_ok=True)
        tail = log_path.read_text(encoding='utf-8', errors='replace')[-1000:]
        print('Recorder failed to start:', tail)
        return 1
    if not (output / 'health.json').exists():
        print('Recorder is still initializing; check status/log before stopping:', output)
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
    summary_path = Path(state['output']) / 'summary.json'
    finalizing = summary_path.exists()
    if not finalizing:
        os.killpg(pid, signal.SIGINT)
    for _ in range(50):
        if not recorder_alive(state):
            break
        time.sleep(0.1)
    if recorder_alive(state):
        print('Finalizing video; source frames are saved. Check status:', state['output'])
        return 0
    STATE.unlink(missing_ok=True)
    if not summary_path.exists():
        print('Recorder exited without a final report; inspect:', state['log'])
        return 1
    print('Recording stopped:', state['output'])
    return 0


def status():
    state = load_state()
    if recorder_alive(state):
        print('Recording active:', state['output'])
        health = Path(state['output']) / 'health.json'
        if health.exists():
            print(health.read_text(encoding='utf-8'))
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
