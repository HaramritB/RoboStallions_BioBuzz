#!/usr/bin/env python3
"""Generate Java geometry and a conservative clearance audit from the saved visualizer file.

Run normally to regenerate; --check fails if generated artifacts are stale. No robot/network IO.
The 141.5-inch reference frame matches the inspected visualizer, not a new physical measurement.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'TeamCode/routes/BioBuzz_AutoPath.pp.json'
JAVA = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/BioBuzzAutoPaths.java'
AUDIT = ROOT / 'TeamCode/routes/BioBuzz_route_audit.json'


def number(x):
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x):
        raise ValueError('Route values must be finite numbers')
    return x


def point(p):
    return number(p['x']), number(p['y'])


def wrap(degrees):
    return (degrees + 180) % 360 - 180


def inside(p, vertices):
    x, y = p
    hit = False
    for a, b in zip(vertices, vertices[1:] + vertices[:1]):
        ax, ay = a
        bx, by = b
        cross = (x - ax) * (by - ay) - (y - ay) * (bx - ax)
        if abs(cross) < 1e-9 and min(ax, bx) <= x <= max(ax, bx) and min(ay, by) <= y <= max(ay, by):
            return True
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            hit = not hit
    return hit


def bezier(points, t):
    points = list(points)
    while len(points) > 1:
        points = [(a[0] * (1-t) + b[0] * t, a[1] * (1-t) + b[1] * t)
                  for a, b in zip(points, points[1:])]
    return points[0]


def generate(raw):
    data = json.loads(raw, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
    if data.get('version') != '2.0.0':
        raise ValueError('Inspect format changes before importing a different visualizer version')
    lines = data['lines']
    ids = [line['id'] for line in lines]
    if not ids or len(set(ids)) != len(ids):
        raise ValueError('Route path IDs must be nonempty and unique')
    order = [item['lineId'] for item in data['sequence'] if item['kind'] == 'path']
    if len(order) != len(data['sequence']) or order != ids:
        raise ValueError('This generator requires each path exactly once, in geometric order; inspect other actions')
    start = point(data['startPoint'])
    initial_heading = number(data['startPoint']['headingDeg'])
    previous, previous_heading = start, initial_heading
    expressions, starts, ends, turns, conflicts = [], [], [], [], []
    shapes = [(shape.get('name', shape['id']), [point(p) for p in shape['vertices']])
              for shape in data.get('shapes', [])]
    samples = []
    for i, line in enumerate(lines, 1):
        if line.get('kind') != 'atomic' or line.get('throughPoints'):
            raise ValueError('Only atomic line/control-point Bezier paths are supported here')
        heading = line['heading']
        if heading['type'] not in ('linear', 'constant'):
            raise ValueError('Explicit waypoint headings are required; no tangent substitution')
        hs = number(heading['startDeg'] if heading['type'] == 'linear' else heading['degrees'])
        he = number(heading['endDeg'] if heading['type'] == 'linear' else heading['degrees'])
        end = point(line['endPoint'])
        if math.dist(previous, end) < 1e-6:
            raise ValueError('Do not encode turns as zero-length drive paths')
        if i == 1 and abs(wrap(hs - initial_heading)) > 1e-6:
            raise ValueError('First path heading must match the configured physical start orientation')
        controls = [point(p) for p in line.get('controlPoints', [])]
        starts.append((*previous, hs))
        ends.append((*end, he))
        if abs(wrap(hs - previous_heading)) > 1e-6:
            turns.append({'before_path': i, 'position': list(previous),
                          'from_degrees': previous_heading, 'to_degrees': hs})
        fmt = lambda values: 'P.of(' + ', '.join(json.dumps(v) for v in values) + ')'
        pts = [f'starts[{i-1}]'] + [fmt((*p, 0)) for p in controls] + [f'ends[{i-1}]']
        geometry = ('line' if not controls else 'curve') + '(' + ', '.join(pts) + ')'
        expressions.append(geometry + (f'.linear(starts[{i-1}], ends[{i-1}])' if heading['type'] == 'linear'
                                       else f'.constant(starts[{i-1}])'))
        hit_shapes = set()
        for k in range(1001):
            t = k / 1000
            p = bezier([previous, *controls, end], t)
            samples.append((i, p, hs + wrap(he-hs) * t))
            for name, vertices in shapes:
                if inside(p, vertices):
                    hit_shapes.add(name)
        end_hits = [name for name, vertices in shapes if inside(end, vertices)]
        if hit_shapes:
            conflicts.append({'path': i, 'sampled_center_intersections': sorted(hit_shapes),
                              'endpoint_inside': end_hits})
        previous, previous_heading = end, he
    digest = hashlib.sha256(raw).hexdigest()
    java = '''package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/** Generated by TeamCode/tools/sync_autonomous_route.py; edit the .pp.json, then regenerate.
 * Inches, +X right, +Y up; headings CCW from +X. Start forward is +Y (90 deg).
 * Geometry is authoritative input, not an assertion of physical clearance or calibration.
 */
public final class BioBuzzAutoPaths {
    public static final String SOURCE_SHA256 = "DIGEST";
    public static final int PATH_COUNT = COUNT;
    private static final PoseFactory P = PoseFactory.degrees();
    public final Pose startPose = START;
    private final Pose[] starts = {STARTS};
    private final Pose[] ends = {ENDS};

    public final Pose flowerTipPose;
    public BioBuzzAutoPaths() { this(0, 0); } // Raw source geometry for inspection/generation.
    public BioBuzzAutoPaths(double forwardIn, double leftIn) {
        if (!Double.isFinite(forwardIn) || forwardIn<0 || !Double.isFinite(leftIn)) {
            throw new IllegalArgumentException("Invalid measured intake offset");
        }
        flowerTipPose=PATH_COUNT>=4 ? ends[3] : null;
        if (forwardIn!=0 || leftIn!=0) {
            if (PATH_COUNT<5) throw new IllegalArgumentException("FLOWER correction requires paths 4 and 5");
            double h=flowerTipPose.heading();
            Pose center=P.of(flowerTipPose.x()-forwardIn*Math.cos(h)+leftIn*Math.sin(h),
                    flowerTipPose.y()-forwardIn*Math.sin(h)-leftIn*Math.cos(h), Math.toDegrees(h));
            ends[3]=center;
            starts[4]=center; // Exact continuity into path 5 after converting tip to center.
        }
    }

    public Pose start(int path) { check(path); return starts[path-1]; }
    public Pose end(int path) { check(path); return ends[path-1]; }
    public boolean turnBefore(int path) {
        check(path);
        return path > 1 && Math.abs(BioBuzzAutoConfig.headingErrorDegrees(start(path), end(path-1))) > 1e-6;
    }
    public Path path(int path) {
        check(path);
        switch (path) {
CASES
            default: throw new IllegalArgumentException("Unknown path");
        }
    }
    private static void check(int path) {
        if (path < 1 || path > PATH_COUNT) throw new IllegalArgumentException("Path must be 1.." + PATH_COUNT);
    }
}
'''.replace('DIGEST', digest)
    java = java.replace('= COUNT;', '= ' + str(len(lines)) + ';')
    java = java.replace('START;', fmt((*start, initial_heading)) + ';')
    java = java.replace('{STARTS}', '{\n        ' + ',\n        '.join(fmt(p) for p in starts) + '\n    }')
    java = java.replace('{ENDS}', '{\n        ' + ',\n        '.join(fmt(p) for p in ends) + '\n    }')
    java = java.replace('CASES', '\n'.join(f'            case {i}: return {expr};' for i, expr in enumerate(expressions, 1)))
    robot = data.get('robot') or {}
    length, width = robot.get('rLength'), robot.get('rWidth')
    footprint_known = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v > 0
                          for v in (length, width))
    footprint_conflicts = []
    if footprint_known:
        # Corner/edge intersections with polygon edges, including obstacle wholly inside robot.
        def intersects(a,b,c,d):
            cross=lambda u,v,w:(v[0]-u[0])*(w[1]-u[1])-(v[1]-u[1])*(w[0]-u[0])
            return cross(a,b,c)*cross(a,b,d)<=0 and cross(c,d,a)*cross(c,d,b)<=0 and \
                max(min(a[0],b[0]),min(c[0],d[0]))<=min(max(a[0],b[0]),max(c[0],d[0])) and \
                max(min(a[1],b[1]),min(c[1],d[1]))<=min(max(a[1],b[1]),max(c[1],d[1]))
        found=set()
        for i,(x,y),h in samples:
            h=math.radians(h)
            corners=[(x+u*math.cos(h)-v*math.sin(h),y+u*math.sin(h)+v*math.cos(h))
                     for u,v in [(-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)]]
            if any(cx<0 or cy<0 or cx>141.5 or cy>141.5 for cx,cy in corners): found.add((i,'reference field boundary'))
            for name,vertices in shapes:
                edges=lambda p:zip(p,p[1:]+p[:1])
                if any(inside(p,vertices) for p in corners) or any(inside(p,corners) for p in vertices) or \
                   any(intersects(a,b,c,d) for a,b in edges(corners) for c,d in edges(vertices)):
                    found.add((i,name))
        footprint_conflicts=[{'path':i,'obstacle':name} for i,name in sorted(found)]
    report = {'source':str(SOURCE.relative_to(ROOT)), 'source_sha256':digest,
              'requested_attachment_name':'BioBuzz_AutoPath.pp.json',
              'input_note':'Raw source geometry is preserved. User clarified waypoint 4 as the intake tip; runtime Java applies measured forward/left offsets to path 4 end and path 5 start.',
              'path_count':len(lines), 'explicit_turns':turns,
              'robot_dimensions_present':footprint_known, 'robot_length_in':length,'robot_width_in':width,
              'clearance_status':'BLOCKED RAW-EXPORT CLEARANCE CERTIFICATION: intake-tip offset and measured footprint not applied in this audit',
              'reference_field_size_in':141.5, 'reference_not_physical_measurement':True,
              'path4_center_to_reference_upper_boundary_in':round(141.5-ends[3][1],4) if len(ends)>=4 else None,
              'center_conflicts':conflicts, 'sampled_footprint_conflicts':footprint_conflicts,
              'sampling_note':'1001 Bezier-parameter samples per path. Not a continuous swept-volume or physical clearance proof.',
              'required_resolution':'Waypoint 4 is an intake-tip target. Supply measured center-to-tip offsets; runtime conversion preserves headings and path continuity. The corrected chassis route still needs physical clearance verification.'}
    return java, json.dumps(report, indent=2, allow_nan=False)+'\n'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    java,audit=generate(SOURCE.read_bytes())
    for path,content in [(JAVA,java),(AUDIT,audit)]:
        if args.check:
            if not path.exists() or path.read_text()!=content:
                raise SystemExit(f'Stale generated artifact: {path.relative_to(ROOT)}; run {Path(__file__).relative_to(ROOT)}')
        else:
            path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(content)
    print(('Verified' if args.check else 'Generated')+' route Java and clearance audit from the saved JSON.')

if __name__=='__main__':
    main()
