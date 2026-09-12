#!/usr/bin/env python3
"""Small always-on-top Windows overlay for FSDS manual/automatic control."""
import argparse
import math
import os
import queue
import shlex
import subprocess
import threading


MODES = ('manual', 'auto', 'stop')
ARROW_KEYS = (0x25, 0x26, 0x27, 0x28)


def mode_after_result(requested_mode, return_code):
    """A failed switch has unknown ownership and must remain retryable."""
    return requested_mode if return_code == 0 else None


def build_speed_invocation(distro, repository, speed, throttle_scale=0.20,
                           steering_gain=0.8, steering_response=0.5,
                           steering_limit=0.7):
    speed = float(speed)
    throttle_scale = float(throttle_scale)
    steering_gain = float(steering_gain)
    steering_response = float(steering_response)
    steering_limit = float(steering_limit)
    if not math.isfinite(speed) or not 0.1 <= speed <= 15.0:
        raise ValueError('Speed must be 0.1..15.0 m/s')
    if not math.isfinite(throttle_scale) or not 0 <= throttle_scale <= 1:
        raise ValueError('Throttle scale must be 0..1')
    if not math.isfinite(steering_gain) or not 0 <= steering_gain <= 3:
        raise ValueError('Steering gain must be 0..3')
    if not math.isfinite(steering_response) or not 0.05 <= steering_response <= 1:
        raise ValueError('Steering response must be 0.05..1')
    if not math.isfinite(steering_limit) or not 0.1 <= steering_limit <= 1:
        raise ValueError('Steering limit must be 0.1..1')
    command = 'cd %s && source /opt/ros/humble/setup.bash && ' % shlex.quote(repository)
    command += 'source simulation_ws/install/setup.bash && '
    command += ('python3 tools/fsds_speed.py %.2f --throttle-scale %.2f '
                '--steering-gain %.2f --steering-response %.2f '
                '--steering-limit %.2f') % (
                    speed, throttle_scale, steering_gain,
                    steering_response, steering_limit)
    return ['wsl.exe', '-d', distro, 'bash', '-lc', command]


def build_record_invocation(distro, repository, action):
    if action not in ('start', 'stop', 'status'):
        raise ValueError('invalid recording action')
    command = 'cd %s && source /opt/ros/humble/setup.bash && ' % shlex.quote(repository)
    command += 'source simulation_ws/install/setup.bash && '
    command += 'python3 tools/fsds_record.py %s' % action
    return ['wsl.exe', '-d', distro, 'bash', '-lc', command]


def build_status_invocation(distro, repository):
    command = 'cd %s && source /opt/ros/humble/setup.bash && ' % shlex.quote(repository)
    command += 'source simulation_ws/install/setup.bash && '
    command += 'timeout 4 ros2 topic echo /car/status --once'
    return ['wsl.exe', '-d', distro, 'bash', '-lc', command]


def build_wsl_invocation(distro, repository, mode):
    if mode not in MODES:
        raise ValueError('invalid mode')
    command = 'cd %s && source /opt/ros/humble/setup.bash && ' % shlex.quote(repository)
    command += 'source simulation_ws/install/setup.bash && '
    command += 'python3 tools/fsds_mode.py %s' % mode
    return ['wsl.exe', '-d', distro, 'bash', '-lc', command]


def build_close_invocations(distro, repository):
    """Return keyboard control first, then finalize any active recording."""
    return (build_wsl_invocation(distro, repository, 'manual'),
            build_record_invocation(distro, repository, 'stop'))


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--distro', default='Ubuntu-22.04')
    parser.add_argument('--repository', default='/home/danis/car_control_project_new_Danis')
    parser.add_argument('--initial-mode', choices=MODES, default='manual')
    args = parser.parse_args(argv)
    if os.name != 'nt':
        parser.error('this overlay runs on Windows; use tools/fsds_mode.py in WSL')

    import ctypes
    import tkinter as tk

    root = tk.Tk()
    root.title('FSDS CONTROL')
    root.attributes('-topmost', True)
    root.resizable(True, True)
    width, height = 420, 650
    root.minsize(360, 500)
    root.geometry('%dx%d+%d+30' % (width, height, root.winfo_screenwidth() - width - 30))

    result_queue = queue.Queue()
    state = {'mode': args.initial_mode, 'busy': False, 'arrows': False,
             'closing': False}
    status = tk.StringVar(value=args.initial_mode.upper())
    detail = tk.StringVar(value='Arrows automatically select MANUAL')

    tk.Label(root, text='FSDS CONTROL', font=('Segoe UI', 13, 'bold')).pack(pady=(8, 2))
    status_label = tk.Label(root, textvariable=status, font=('Segoe UI', 11, 'bold'))
    status_label.pack()
    buttons = tk.Frame(root)
    buttons.pack(pady=5)

    def finish(mode, return_code, output):
        result_queue.put((mode, return_code, output.strip()))

    def run_async(kind, invocation):
        def worker():
            try:
                flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
                timeout = 60 if kind in MODES else 20
                result = subprocess.run(
                    invocation, text=True, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, creationflags=flags,
                    timeout=timeout, check=False)
                finish(kind, result.returncode, result.stdout)
            except Exception as error:
                finish(kind, 1, str(error))
        threading.Thread(target=worker, daemon=True).start()

    def request(mode, force=False):
        if state['busy'] or (state['mode'] == mode and not force):
            return
        if mode == 'auto' and state['arrows']:
            detail.set('Release arrow keys before enabling AUTOPILOT')
            return
        state['busy'] = True
        status.set('SWITCHING…')
        detail.set('Waiting for ROS2/FSDS acknowledgement')

        run_async(mode, build_wsl_invocation(args.distro, args.repository, mode))

    tk.Button(buttons, text='MANUAL', width=9, command=lambda: request('manual')).pack(side='left', padx=3)
    tk.Button(buttons, text='AUTOPILOT', width=9, command=lambda: request('auto')).pack(side='left', padx=3)
    tk.Button(buttons, text='STOP', width=9, command=lambda: request('stop'), bg='#ffb3b3').pack(side='left', padx=3)
    speed = tk.DoubleVar(value=2.5)
    throttle_scale = tk.DoubleVar(value=0.20)
    steering_gain = tk.DoubleVar(value=0.80)
    steering_response = tk.DoubleVar(value=0.50)
    steering_limit = tk.DoubleVar(value=0.70)
    speed_status = tk.StringVar(value='Target not applied; running launch keeps its value')
    speed_busy = [False]

    def apply_speed():
        if speed_busy[0]:
            return
        invocation = build_speed_invocation(
            args.distro, args.repository, speed.get(), throttle_scale.get(),
            steering_gain.get(), steering_response.get(), steering_limit.get())
        speed_busy[0] = True
        speed_status.set('Applying target...')

        run_async('speed', invocation)

    tk.Scale(root, from_=0.1, to=15.0, resolution=0.1, orient='horizontal',
             variable=speed, label='Autopilot target (m/s)', length=360).pack(
                 fill='x', padx=12)
    tk.Scale(root, from_=0.05, to=0.50, resolution=0.01, orient='horizontal',
             variable=throttle_scale, label='Maximum FSDS throttle', length=360).pack(
                 fill='x', padx=12)
    tk.Scale(root, from_=0.0, to=3.0, resolution=0.05, orient='horizontal',
             variable=steering_gain,
             label='Turn sharpness / steering gain (Kp)', length=380).pack(
                 fill='x', padx=12)
    tk.Scale(root, from_=0.05, to=1.0, resolution=0.05, orient='horizontal',
             variable=steering_response,
             label='Turn response speed (EMA; higher = faster)', length=380).pack(
                 fill='x', padx=12)
    tk.Scale(root, from_=0.1, to=1.0, resolution=0.05, orient='horizontal',
             variable=steering_limit,
             label='Maximum steering command', length=380).pack(
                 fill='x', padx=12)
    tk.Button(root, text='Apply driving settings', command=apply_speed).pack()
    tk.Label(root, textvariable=speed_status, wraplength=340, font=('Segoe UI', 8)).pack()
    tk.Label(root, text='Perception: FSDS ground truth cones',
             font=('Segoe UI', 8, 'italic')).pack(pady=(4, 0))

    recording = tk.StringVar(value='Recording: unknown')
    record_buttons = tk.Frame(root)
    record_buttons.pack(pady=4)

    def record(action):
        recording.set('Recording: %s...' % action)
        run_async('record', build_record_invocation(
            args.distro, args.repository, action))

    tk.Button(record_buttons, text='START RECORD', width=13,
              command=lambda: record('start')).pack(side='left', padx=3)
    tk.Button(record_buttons, text='STOP RECORD', width=13,
              command=lambda: record('stop')).pack(side='left', padx=3)
    debug = tk.StringVar(value='Status: press Refresh')
    tk.Button(root, text='Refresh status', command=lambda: run_async(
        'debug', build_status_invocation(args.distro, args.repository))).pack()
    tk.Label(root, textvariable=recording, wraplength=380,
             font=('Segoe UI', 8)).pack()
    tk.Label(root, textvariable=debug, wraplength=380,
             font=('Consolas', 8)).pack()
    tk.Label(root, textvariable=detail, wraplength=315, font=('Segoe UI', 8)).pack()

    def poll():
        try:
            while True:
                mode, return_code, output = result_queue.get_nowait()
                if mode == 'speed':
                    speed_busy[0] = False
                    speed_status.set(output[-180:] if output else 'No acknowledgement')
                    continue
                if mode == 'record':
                    recording.set(output.splitlines()[-1] if output else
                                  'Recording command failed')
                    continue
                if mode == 'debug':
                    debug.set(output[-300:] if output else 'Status unavailable')
                    continue
                state['busy'] = False
                if return_code == 0:
                    state['mode'] = mode_after_result(mode, return_code)
                    status.set(mode.upper())
                    status_label.configure(fg={'manual': '#b36b00', 'auto': '#008000', 'stop': '#b00020'}[mode])
                    detail.set(output.splitlines()[-1] if output else 'Mode changed')
                else:
                    state['mode'] = mode_after_result(mode, return_code)
                    status.set('ERROR')
                    status_label.configure(fg='#b00020')
                    detail.set(output[-180:] if output else 'Mode switch failed')
        except queue.Empty:
            pass

        arrows = any(ctypes.windll.user32.GetAsyncKeyState(key) & 0x8000 for key in ARROW_KEYS)
        if arrows and not state['arrows']:
            request('manual', force=True)
        state['arrows'] = arrows
        root.after(50, poll)

    def close_panel():
        if state['closing']:
            return
        state['closing'] = True
        status.set('RETURNING MANUAL…')
        detail.set('Stopping recording and returning keyboard control')

        def worker():
            flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
            for invocation in build_close_invocations(
                    args.distro, args.repository):
                try:
                    subprocess.run(invocation, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, creationflags=flags,
                                   timeout=45, check=False)
                except Exception:
                    pass
            root.after(0, root.destroy)
        threading.Thread(target=worker, daemon=True).start()

    root.protocol('WM_DELETE_WINDOW', close_panel)
    poll()
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
