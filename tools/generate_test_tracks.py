#!/usr/bin/env python3
"""Generate deterministic FSDS v2.2.0 CustomMap CSVs in metres (Y left).

No Unreal editor, third-party geometry libraries or changes to vehicle physics.
"""
import argparse
import csv
import json
import math
from pathlib import Path


def resample(points, spacing=2.0, closed=False):
    points = list(points)
    if closed and points[-1] != points[0]:
        points.append(points[0])
    result = [points[0]]
    remainder = spacing
    for a, b in zip(points, points[1:]):
        length = math.dist(a, b)
        if length == 0:
            continue
        while remainder <= length:
            ratio = remainder / length
            result.append((a[0] + ratio*(b[0]-a[0]), a[1] + ratio*(b[1]-a[1])))
            remainder += spacing
        remainder -= length
    if closed and math.dist(result[-1], result[0]) < spacing * .5:
        result.pop()
    elif not closed and math.dist(result[-1], points[-1]) > .25:
        result.append(points[-1])
    return result


def normalize(points):
    origin = points[0]
    angle = math.atan2(points[1][1]-origin[1], points[1][0]-origin[0])
    c, s = math.cos(angle), math.sin(angle)
    return [((x-origin[0])*c+(y-origin[1])*s,
             -(x-origin[0])*s+(y-origin[1])*c) for x, y in points]


def centerlines():
    figure = [(30*math.sin(t), 16*math.sin(2*t))
              for t in [-math.pi/2 + i*2*math.pi/2000 for i in range(2000)]]
    slalom = [(i*.05, 4*math.sin(2*math.pi*i*.05/30)**3) for i in range(1801)]
    # Rounded rectangle: four 90-degree bends joined by straight sections.
    turns = []
    for cx, cy, start in [(52, 8, -90), (52, 32, 0), (8, 32, 90), (8, 8, 180)]:
        turns.extend((cx+8*math.cos(math.radians(start+i*.5)),
                      cy+8*math.sin(math.radians(start+i*.5))) for i in range(181))
    return {'figure_eight': (normalize(resample(figure, closed=True)), True),
            'slalom': (normalize(resample(slalom)), False),
            'turns': (normalize(resample(turns, closed=True)), True)}


def cones_for(points, closed, width=4.0, crossing=False):
    cones = []
    n = len(points)
    for i, (x, y) in enumerate(points):
        before = points[(i-1) % n] if closed or i else points[0]
        after = points[(i+1) % n] if closed or i+1 < n else points[-1]
        dx, dy = after[0]-before[0], after[1]-before[1]
        length = math.hypot(dx, dy)
        for tag, sign in [('blue', 1), ('yellow', -1)]:
            cone = (x-sign*dy/length*width/2, y+sign*dx/length*width/2)
            # Keep the crossing physically clear of cones from the other arm.
            if crossing and any(math.dist(cone, p) < width/2+.35
                for j, p in enumerate(points) if min(abs(i-j), n-abs(i-j)) > 8):
                continue
            # Reserve space for the start gate.
            if math.hypot(cone[0], cone[1]) < width/2 + .8:
                continue
            cones.append((tag, *cone))
    # Loader needs exactly two or four orange cones, otherwise finish computation is invalid.
    # Upstream computes atan2(UE_y0-UE_y1, x0-x1)-pi/2; right first gives +X.
    cones.extend([('big_orange', 0., -width/2), ('big_orange', 0., width/2)])
    return cones


def write_track(output, name, points, cones, closed, description):
    with (output / (name+'.csv')).open('w', newline='', encoding='utf-8') as handle:
        csv.writer(handle, lineterminator='\n').writerows(
            (tag, '%.5f' % x, '%.5f' % y, 0, 0, 0, 0) for tag, x, y in cones)
    metadata = {'name': name, 'units': 'metres', 'coordinates': 'FSDS CSV: X forward, Y left',
                'closed': closed, 'width_m': 4., 'nominal_cone_spacing_m': 2.,
                'centerline': points, 'description': description,
                'start_gate': [0., 0.], 'intended_start_heading_rad': 0.,
                'length_m': sum(math.dist(a,b) for a,b in zip(points, points[1:]))
                            + (math.dist(points[-1], points[0]) if closed else 0.)}
    (output / (name+'.json')).write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    xs, ys = [c[1] for c in cones], [c[2] for c in cones]
    lowx, highx, lowy, highy = min(xs)-5, max(xs)+5, min(ys)-5, max(ys)+5
    colors = {'blue':'#1976d2', 'yellow':'#e2ac00', 'big_orange':'#ff6500'}
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="%s %s %s %s">' %
           (lowx, -highy, highx-lowx, highy-lowy),
           '<rect x="%s" y="%s" width="100%%" height="100%%" fill="#f4f6f8"/>' % (lowx,-highy)]
    svg.extend('<circle cx="%.4f" cy="%.4f" r=".35" fill="%s"/>' % (x,-y,colors[tag])
               for tag,x,y in cones)
    svg.append('<path d="M 0 0 L 4 0 L 3 -0.6 M 4 0 L 3 0.6" stroke="#111" stroke-width=".25" fill="none"/>')
    svg.append('</svg>')
    (output / (name+'.svg')).write_text('\n'.join(svg), encoding='utf-8')


def generate(output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    combined = []
    descriptions = {
        'figure_eight': 'Figure eight with an open intersection. Route ambiguity at the crossing requires memory/planning; cone-only reactive control may choose the wrong branch.',
        'slalom': 'Open 90 m slalom, amplitude 4 m, period 30 m. Stop manually before the end; not a closed lap.',
        'turns': 'Closed rounded rectangle, four 90 degree turns of radius 8 m.'}
    zones = []
    for index, (name, (points, closed)) in enumerate(centerlines().items()):
        cones = cones_for(points, closed, crossing=name=='figure_eight')
        write_track(output, name, points, cones, closed, descriptions[name])
        offset = (index*130., 0.)
        for tag,x,y in cones:
            # Only the first zone owns the map-level orange start/finish gate.
            if tag == 'big_orange' and index:
                tag = 'blue' if y > 0 else 'yellow'
            combined.append((tag,x+offset[0],y+offset[1]))
        zones.append({'scenario': name, 'offset_m': offset, 'centerline':
                      [(x+offset[0],y+offset[1]) for x,y in points]})
    write_track(output, 'test_ground', [(0.,0.)], combined, False,
                'Three separate test zones: figure eight, slalom, turns. Transfer manually; not one connected route.')
    (output/'zones.json').write_text(json.dumps(zones, indent=2), encoding='utf-8')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1]/'simulation/tracks')
    generate(parser.parse_args().output)
