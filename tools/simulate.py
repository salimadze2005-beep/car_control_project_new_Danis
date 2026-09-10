"""Native Windows/Linux deterministic runner using the existing Bicycle and core."""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'shared'))
from car_control_core.core import Bicycle, Controller, Parameters, circle_track, world_to_cones


def run(seconds=240.0, gui=False):
    car, controller, track = Bicycle(), Controller(Parameters()), circle_track()
    dt, worst = 0.02, 0.0
    canvas = root = None
    if gui:
        import tkinter as tk
        root = tk.Tk()
        root.title('Shared controller — lightweight simulation')
        canvas = tk.Canvas(root, width=720, height=620, bg='#18242d')
        canvas.pack()
        tk.Label(root, text='Blue/yellow boundaries · green car · shared PID core · close window to stop').pack()
        for x, y, color in track:
            px, py = 360 + x*28, 540-y*28
            canvas.create_oval(px-3, py-3, px+3, py+3, fill=color, outline='')
        marker = canvas.create_oval(0,0,0,0,fill='#36e09a')
    def step():
        nonlocal worst
        command = controller.step(world_to_cones(track, car.x, car.y, car.yaw, 4.), dt)
        car.step(command, dt)
        worst = max(worst, abs(math.hypot(car.x, car.y-8.) - 8.))
    if gui:
        count = [0]
        def animate():
            step()
            count[0] += 1
            px, py = 360+car.x*28, 540-car.y*28
            canvas.coords(marker, px-5, py-5, px+5, py+5)
            if count[0]*dt < seconds:
                root.after(20, animate)
        root.after(0, animate)
        root.mainloop()
    else:
        for _ in range(int(seconds/dt)):
            step()
    return {'seconds_requested': seconds, 'heading_laps': car.yaw/(2*math.pi),
            'max_cross_track_error_m': worst, 'x_m': car.x, 'y_m': car.y}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=240.)
    parser.add_argument('--gui', action='store_true')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    print(json.dumps(run(args.seconds, args.gui), indent=2))
