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


def build_speed_invocation(distro, repository, speed):
    speed = float(speed)
    if not math.isfinite(speed) or not 0.1 <= speed <= 5.0:
        raise ValueError('Speed must be 0.1..5.0 m/s')
    command = 'cd %s && source /opt/ros/humble/setup.bash && ' % shlex.quote(repository)
    command += 'source simulation_ws/install/setup.bash && '
    command += 'python3 tools/fsds_speed.py %.2f' % speed
    return ['wsl.exe', '-d', distro, 'bash', '-lc', command]


def build_wsl_invocation(distro, repository, mode):
    if mode not in MODES:
        raise ValueError('invalid mode')
    command = 'cd %s && source /opt/ros/humble/setup.bash && ' % shlex.quote(repository)
    command += 'source simulation_ws/install/setup.bash && '
    command += 'python3 tools/fsds_mode.py %s' % mode
    return ['wsl.exe', '-d', distro, 'bash', '-lc', command]


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
    root.resizable(False, False)
    width, height = 360, 260
    root.geometry('%dx%d+%d+30' % (width, height, root.winfo_screenwidth() - width - 30))

    result_queue = queue.Queue()
    state = {'mode': args.initial_mode, 'busy': False, 'arrows': False}
    status = tk.StringVar(value=args.initial_mode.upper())
    detail = tk.StringVar(value='Arrows automatically select MANUAL')

    tk.Label(root, text='FSDS CONTROL', font=('Segoe UI', 13, 'bold')).pack(pady=(8, 2))
    status_label = tk.Label(root, textvariable=status, font=('Segoe UI', 11, 'bold'))
    status_label.pack()
    buttons = tk.Frame(root)
    buttons.pack(pady=5)

    def finish(mode, return_code, output):
        result_queue.put((mode, return_code, output.strip()))

    def request(mode):
        if state['busy'] or state['mode'] == mode:
            return
        state['busy'] = True
        status.set('SWITCHING…')
        detail.set('Waiting for ROS2/FSDS acknowledgement')

        def worker():
            flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
            result = subprocess.run(
                build_wsl_invocation(args.distro, args.repository, mode),
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                creationflags=flags, check=False)
            finish(mode, result.returncode, result.stdout)

        threading.Thread(target=worker, daemon=True).start()

    tk.Button(buttons, text='MANUAL', width=9, command=lambda: request('manual')).pack(side='left', padx=3)
    tk.Button(buttons, text='AUTOPILOT', width=9, command=lambda: request('auto')).pack(side='left', padx=3)
    tk.Button(buttons, text='STOP', width=9, command=lambda: request('stop'), bg='#ffb3b3').pack(side='left', padx=3)
    speed = tk.DoubleVar(value=2.5)
    speed_status = tk.StringVar(value='Target not applied; running launch keeps its value')
    speed_busy = [False]

    def apply_speed():
        if speed_busy[0]:
            return
        invocation = build_speed_invocation(args.distro, args.repository, speed.get())
        speed_busy[0] = True
        speed_status.set('Applying target...')

        def worker():
            try:
                result = subprocess.run(invocation, text=True, stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT, creationflags=subprocess.CREATE_NO_WINDOW,
                    timeout=20, check=False)
                finish('speed', result.returncode, result.stdout)
            except Exception as error:
                finish('speed', 1, str(error))
        threading.Thread(target=worker, daemon=True).start()

    tk.Scale(root, from_=0.1, to=5.0, resolution=0.1, orient='horizontal',
             variable=speed, label='Autopilot target (m/s)', length=320).pack()
    tk.Button(root, text='Apply speed', command=apply_speed).pack()
    tk.Label(root, textvariable=speed_status, wraplength=340, font=('Segoe UI', 8)).pack()
    tk.Label(root, textvariable=detail, wraplength=315, font=('Segoe UI', 8)).pack()

    def poll():
        try:
            while True:
                mode, return_code, output = result_queue.get_nowait()
                if mode == 'speed':
                    speed_busy[0] = False
                    speed_status.set(output[-180:] if output else 'No acknowledgement')
                    continue
                state['busy'] = False
                if return_code == 0:
                    state['mode'] = mode
                    status.set(mode.upper())
                    status_label.configure(fg={'manual': '#b36b00', 'auto': '#008000', 'stop': '#b00020'}[mode])
                    detail.set(output.splitlines()[-1] if output else 'Mode changed')
                else:
                    status.set('ERROR')
                    status_label.configure(fg='#b00020')
                    detail.set(output[-180:] if output else 'Mode switch failed')
        except queue.Empty:
            pass

        arrows = any(ctypes.windll.user32.GetAsyncKeyState(key) & 0x8000 for key in ARROW_KEYS)
        if arrows and not state['arrows'] and state['mode'] != 'manual':
            request('manual')
        state['arrows'] = arrows
        root.after(50, poll)

    poll()
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
