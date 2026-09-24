#!/usr/bin/env python3
"""Launch a generated CustomMap on Windows with an explicit camera profile."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import shutil
import subprocess


def renderer_arguments(renderer='auto', submit_mode='safe'):
    arguments = [] if renderer == 'auto' else ['-' + renderer]
    if renderer != 'd3d11' and submit_mode == 'safe':
        # Must take effect before the first frame. ExecCmds runs too late for
        # the startup VK_ERROR_DEVICE_LOST seen with UE4.27 on Radeon Vega.
        arguments.append('-ini:Engine:[ConsoleVariables]:r.Vulkan.SubmitAfterEveryEndRenderPass=1')
    return arguments


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track', choices=['test_ground','figure_eight','slalom','turns'], default='test_ground')
    parser.add_argument('--fsds-directory', type=Path, default=Path('C:/FSDS'))
    parser.add_argument('--distro', default='Ubuntu-22.04')
    parser.add_argument('--bind-address')
    parser.add_argument('--renderer', choices=['auto', 'd3d11', 'vulkan'], default='auto',
        help='Usually leave auto. This FSDS package has no DirectX 11 shader library.')
    parser.add_argument('--low-graphics', action='store_true', help='Reduce rendering load on integrated GPUs')
    parser.add_argument('--vulkan-submit-mode', choices=['safe', 'default'], default='safe',
        help='safe avoids the observed Radeon/UE4 startup crash; default is for comparison only')
    args = parser.parse_args()
    if os.name != 'nt':
        parser.error('Run this launcher with Windows Python, not WSL Python')
    processes = subprocess.check_output(['tasklist', '/FO', 'CSV', '/NH'], text=True)
    if re.search(r'^"(?:FSDS|Blocks)\.exe"', processes, re.I | re.M):
        parser.error('FSDS is already running; close it before changing maps')
    root = Path(__file__).resolve().parents[1]
    source = root/'simulation/tracks'/(args.track+'.csv')
    exe = args.fsds_directory/'FSDS.exe'
    if not source.is_file() or not exe.is_file():
        parser.error('Missing generated CSV or FSDS.exe')
    address = args.bind_address
    if not address:
        route = subprocess.check_output(['wsl.exe', '-d', args.distro, '--', 'ip', 'route', 'show', 'default'], text=True)
        match = re.search(r'default via ([0-9.]+)', route)
        address = match[1] if match else '127.0.0.1'
    output = args.fsds_directory/'test_runs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    local_map = output/source.name
    shutil.copy2(source, local_map)
    settings = json.loads((root/'simulation/fsds-settings.json').read_text(encoding='utf-8'))
    settings['LocalHostIp'] = address
    settings_path = output/'settings.json'
    settings_path.write_text(json.dumps(settings, indent=2), encoding='utf-8')
    command = [str(exe), '-CustomMapPath='+str(local_map), '-settings', str(settings_path),
               '-windowed', '-ResX=960', '-ResY=640']
    command.extend(renderer_arguments(args.renderer, args.vulkan_submit_mode))
    if args.low_graphics:
        command.extend(['-ResX=640', '-ResY=480',
            '-ExecCmds=sg.ViewDistanceQuality 0,sg.ShadowQuality 0,sg.PostProcessQuality 0,sg.EffectsQuality 0,sg.FoliageQuality 0,r.ScreenPercentage 60'])
    # FSDS is the interactive simulator requested by the user, not a hidden helper.
    process = subprocess.Popen(command, cwd=args.fsds_directory)
    (output/'launch.json').write_text(json.dumps({'pid': process.pid, 'command': command,
        'host': address, 'track': args.track}, indent=2), encoding='utf-8')
    print('FSDS PID=%d; track=%s; host=%s' % (process.pid,args.track,address))
    print('Run files:', output)
    print('WSL: ros2 launch car_control_sim fsds_drive.launch.py host:='+address)


if __name__ == '__main__':
    main()
