#!/usr/bin/env python3
"""Replay real simple-sim raw scans through the production C++ estimator.

Truth is used exclusively to place the robot/scene and score JSON outputs. The
stdin replay never contains arm labels or truth pose observations. Gaussian
fixed-association consistency is a separate C++ experiment, not inferred here.
"""
import argparse
from dataclasses import asdict, dataclass
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
SIMULATOR = PACKAGE.parents[1] / 'simulator' / 'simple-sim'
MOTIONS = ('stationary', 'approach', 'reverse', 'lateral', 'turn')
VIEWS = ('front', 'rear', 'side')


@dataclass(frozen=True)
class Trial:
    kind: str
    motion: str
    view: str
    seed: int
    distance: float = None

    @property
    def name(self):
        name = '{}_{}_{}_{}'.format(self.kind, self.motion, self.view, self.seed)
        return name if self.distance is None else name+'_{}m'.format(self.distance)


def make_trials(seeds=30, adverse_seeds=3, profile='legacy'):
    if profile == 'mir_v150':
        distances = (.7, 1., 1.5, 2., 2.5, 3.)
        trials = [Trial('nominal', MOTIONS[(i//6)%5], 'front', 1000+i, distances[i%6]) for i in range(seeds)]
    else:
        trials = [Trial('nominal', MOTIONS[(i // 3) % 5], VIEWS[i % 3], 1000+i)
              for i in range(seeds)]
    for i in range(adverse_seeds):
        trials += [Trial(kind, 'approach' if kind in ('dropout', 'stale', 'swept') else 'stationary',
                         'front', 2000+i, 2. if profile == 'mir_v150' else None) for kind in
                   ('dropout', 'partial', 'clutter', 'delayed', 'stale', 'swept', 'noiseless', 'overlong')]
    return trials


def wrap(angle):
    return (angle + math.pi) % (2*math.pi) - math.pi


def advance(pose, twist, dt):
    """Exact body-twist SE(2) exponential, independent of estimator code."""
    x, y, yaw = pose
    vx, vy, omega = twist
    a = omega*dt
    sinc = math.sin(a)/a if abs(a) > 1e-8 else 1-a*a/6
    cosc = (1-math.cos(a))/a if abs(a) > 1e-8 else a/2-a**3/24
    dx, dy = dt*(sinc*vx-cosc*vy), dt*(cosc*vx+sinc*vy)
    return np.array([x+math.cos(yaw)*dx-math.sin(yaw)*dy,
                     y+math.sin(yaw)*dx+math.cos(yaw)*dy, wrap(yaw+a)])


def simulator_api():
    # Native kernels and matplotlib must never write inside the existing sim.
    os.environ['XDG_CACHE_HOME'] = '/tmp/vshape-validation-cache'
    os.environ['MPLCONFIGDIR'] = '/tmp/vshape-validation-matplotlib'
    sys.path.insert(0, str(SIMULATOR)) if str(SIMULATOR) not in sys.path else None
    from engine.core import Scene, VMarker, Wall
    from engine.sim.scanner import Scanner, SensorConfig, NoiseConfig
    return Scene, VMarker, Wall, Scanner, SensorConfig, NoiseConfig


def generate_trial(trial, duration=6., n_rays=None, profile='legacy'):
    small = profile == 'mir_v150'
    n_rays = n_rays or (1651 if small else 1441)
    Scene, VMarker, Wall, Scanner, SensorConfig, NoiseConfig = simulator_api()
    robot_yaw = dict(front=math.pi, rear=0., side=math.pi/2)[trial.view]
    lidar_yaw = wrap(math.pi-robot_yaw)
    distance = (.7 if (trial.seed-1000)//15 == 0 else 3.) if trial.kind == 'nominal' and trial.motion == 'stationary' else 2.
    if trial.distance is not None:
        distance = .362+math.sqrt(trial.distance**2-.272**2) if small else trial.distance
    pose0 = np.array([distance, 0., wrap(robot_yaw)])
    twist = np.array(dict(stationary=(0., 0., 0.), approach=(.12, 0., 0.),
                         reverse=(-.10, 0., 0.), lateral=(0., .06, 0.),
                         turn=(.03, 0., .065))[trial.motion])
    # Approach/reverse always move radially irrespective of the viewing direction.
    if trial.motion in ('approach', 'reverse'):
        speed = -.12 if trial.motion == 'approach' else .10
        twist[:2] = [speed*math.cos(robot_yaw), -speed*math.sin(robot_yaw)]
    if small:
        twist *= .25
    if trial.kind == 'swept':
        twist = np.array([.03 if small else .12, 0., 0.])  # scanner's swept model is exact for straight motion
    if small:
        lidar_yaw = -math.pi/4
    sensor = SensorConfig('validation', .362 if small else 0., -.272 if small else 0., lidar_yaw, n_rays=n_rays,
                          fov_deg=270. if small else 180., scan_hz=20., range_max=12., z=.263 if small else .2)
    noise = NoiseConfig(range_sigma_m=0. if trial.kind == 'noiseless' else .01)
    arm_length = .15 if small else .5
    scene = Scene(markers=[VMarker('target', 0., 0., 0., arm_length=2*arm_length if trial.kind == 'overlong' else arm_length,
                                   opening_angle_deg=120. if small else 70., z_min=.213 if small else 0., z_max=.313 if small else 2.)])
    if trial.kind == 'clutter':
        scene.walls = [Wall('background', -1., -2., -1., 2.),
                       Wall('side_clutter', -.5, .8, 1.3, .8),
                       Wall('diagonal_clutter', -.8, -1.5, .8, -.8)]
        scene.markers.append(VMarker('distractor', -.4, -1.3, .2))
    scanner = Scanner(sensor, noise, seed=trial.seed, backend='cpu')
    scanner.sweep = trial.kind == 'swept'
    rng = np.random.default_rng(trial.seed+50000)
    target = [-distance*math.cos(robot_yaw), distance*math.sin(robot_yaw), wrap(-robot_yaw)]
    config = dict(lidar_x=sensor.x, lidar_y=sensor.y, lidar_yaw=lidar_yaw,
                  target_x=target[0], target_y=target[1], target_yaw=target[2],
                  range_sigma=max(noise.range_sigma_m, .0001), angle_sigma=.00001,
                  history_sec=1., odom_timeout=.2, max_events=5000,
                  max_scan_points=20000, max_points_per_arm=24)
    if small:
        config.update(arm_length=.15, opening_angle_deg=120., min_support=.045, endpoint_margin=.01, association_distance=.04, min_points=5)
    lines = ['C {} {:.17g}'.format(key, value) for key, value in config.items()]
    if trial.kind == 'noiseless':
        prior = advance(pose0, [.1, 0., math.radians(5)], 1.)
        lines.append('I 0 {} {} {} .15 .15'.format(*prior))
    events, truth, finite_counts = [], {}, []
    dt = .05
    # Bias is common to both the tracker and odometry-only baseline.
    bias = np.array([.004, -.002, .002]) if trial.kind != 'noiseless' else np.zeros(3)
    for k in range(int(round(duration/dt))+1):
        stamp = round(k*dt, 9)
        pose = advance(pose0, twist, stamp)
        truth[stamp] = pose
        if not (trial.kind == 'stale' and 2. <= stamp < 4.):
            measured = twist+bias+rng.normal(0, [.002, .002, .001]) if trial.kind != 'noiseless' else twist
            events.append((stamp, 0, 'O {:.9f} {:.12g} {:.12g} {:.12g}'.format(stamp, *measured)))
        if not (trial.kind in ('dropout', 'stale') and 2. <= stamp < 4.):
            scan = scanner.scan(scene, pose, motion=(float(twist[0]), float(twist[2])))
            ranges = scan.ranges.copy()
            angles = scan.angle_min + np.arange(len(ranges))*scan.angle_increment
            if trial.kind == 'partial' and 2. <= stamp < 4.:
                center = wrap(math.atan2(-pose0[1]-math.sin(robot_yaw)*sensor.x-math.cos(robot_yaw)*sensor.y, -pose0[0]-math.cos(robot_yaw)*sensor.x+math.sin(robot_yaw)*sensor.y)-robot_yaw-lidar_yaw) if small else 0.
                ranges[(angles >= center-.01) & (angles <= center+.5)] = np.inf
            if trial.kind == 'clutter':
                outliers = rng.random(len(ranges)) < .015
                ranges[outliers] = rng.uniform(.3, 5., outliers.sum())
            arrival = stamp + (scan.time_increment*(len(ranges)-1) if scanner.sweep else 0.)
            if trial.kind == 'delayed' and k % 6 == 0:
                arrival += .15
            event = 'S {:.9f} {:.17g} {:.17g} {:.17g} {} {}'.format(
                stamp, scan.angle_min, scan.angle_increment, scan.time_increment, len(ranges),
                ' '.join('{:.9g}'.format(r) for r in ranges))
            if arrival <= duration+1e-8:
                events.append((round(arrival, 9), 1, event))
                finite_counts.append(int(np.isfinite(ranges).sum()))
        events.append((stamp, 2, 'P {:.9f}'.format(stamp)))
    events.sort(key=lambda event: (event[0], event[1]))
    lines += [event[2] for event in events]
    metadata = dict(trial=asdict(trial), profile=profile, duration=duration, dt=dt,
                    initial_lidar_range_m=float(np.linalg.norm(pose0[:2]+np.array([[math.cos(robot_yaw),-math.sin(robot_yaw)],[math.sin(robot_yaw),math.cos(robot_yaw)]])@np.array([sensor.x,sensor.y]))),
                    finite_returns_per_delivered_scan=dict(min=min(finite_counts), median=float(np.median(finite_counts)), max=max(finite_counts)),
                    sensor=asdict(sensor), angular_resolution_deg=sensor.fov_deg/(sensor.n_rays-1), noise=asdict(noise), config=config,
                    scene=[asdict(obj) for obj in scene.objects()],
                    truth_initial=pose0.tolist(), truth_twist=twist.tolist(),
                    odom_bias=bias.tolist(), scanner='engine.sim.scanner.Scanner',
                    swept=scanner.sweep, first_ray_reference=True,
                    input_sha256=hashlib.sha256(('\n'.join(lines)+'\n').encode()).hexdigest())
    return '\n'.join(lines)+'\n', truth, metadata


def parse_outputs(stdout, expected_stamps):
    rows = [json.loads(line) for line in stdout.splitlines() if line.strip()]
    required = {'stamp', 'initialized', 'pose', 'covariance', 'mode', 'reason', 'last_measurement',
                'odom_age', 'nis', 'dof', 'points', 'eigenvalues', 'updates', 'rejected', 'replays', 'update_ms'}
    if len(rows) != len(expected_stamps):
        raise ValueError('expected {} output rows, got {}'.format(len(expected_stamps), len(rows)))
    for row, stamp in zip(rows, expected_stamps):
        if not required <= row.keys() or abs(row['stamp']-stamp) > 1e-7:
            raise ValueError('invalid output contract at stamp {}'.format(stamp))
        if len(row['pose']) != 3 or len(row['covariance']) != 9:
            raise ValueError('invalid pose/covariance dimensions')
    return rows


def replay(binary, inputs, capture_path=None):
    start = time.perf_counter()
    proc = subprocess.run([str(binary)], input=inputs, text=True, capture_output=True, timeout=120)
    if capture_path is not None:
        capture_path.write_text(proc.stdout)
        capture_path.with_suffix('.stderr').write_text(proc.stderr)
    if proc.returncode:
        raise RuntimeError('replay exit {}: {}'.format(proc.returncode, proc.stderr[-2000:]))
    stamps = [float(line.split()[1]) for line in inputs.splitlines() if line.startswith('P ')]
    return parse_outputs(proc.stdout, stamps), proc.stdout, time.perf_counter()-start


def baseline_inputs(inputs, outputs):
    first = next((row for row in outputs if row['initialized']), None)
    if first is None:
        raise ValueError('tracker never initialized; odometry baseline cannot share its posterior')
    stamp = first['stamp']
    lines = [line for line in inputs.splitlines() if line.startswith('C ')]
    covariance = np.array(first['covariance']).reshape(3, 3)
    lines.append('I {:.9f} {:.17g} {:.17g} {:.17g} {:.17g} {:.17g}'.format(
        stamp, *first['pose'], math.sqrt(max(covariance[0,0], 1e-12)),
        math.sqrt(max(covariance[2,2], 1e-12))))
    odom = [line for line in inputs.splitlines() if line.startswith('O ')]
    previous = [line for line in odom if float(line.split()[1]) <= stamp]
    if previous:
        lines.append('O {:.9f} {}'.format(stamp, ' '.join(previous[-1].split()[2:])))
    lines += [line for line in inputs.splitlines() if line.startswith(('O ', 'P '))
              and float(line.split()[1]) > stamp]
    return '\n'.join(lines)+'\n'


def chronological_inputs(inputs):
    lines = inputs.splitlines()
    prefix = [line for line in lines if line.startswith(('C ', 'I '))]
    events = [line for line in lines if line.startswith(('O ', 'S ', 'P '))]
    events.sort(key=lambda line: (float(line.split()[1]), {'O': 0, 'S': 1, 'P': 2}[line[0]]))
    return '\n'.join(prefix+events)+'\n'


def output_to_mouth(rows, truth, depth=.075):
    """Match ROS openingFacePose/DockingConvention and additive covariance.

    Input covariance is the core robot-in-marker right tangent. Each output
    pose is the opening-face midpoint in the robot base, with the outward
    heading branch chosen once at acquisition, as in the production node.
    """
    def pose_at_mouth(pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c*(depth-x)-s*y, -s*(depth-x)-c*y, wrap(-yaw)])

    def branch(pose):
        return math.pi if math.cos(pose[2])*pose[0]+math.sin(pose[2])*pose[1] < 0 else 0.

    initialized = next((row for row in rows if row['initialized']), None)
    truth_stamp = round(initialized['stamp'], 9) if initialized else min(truth)
    truth_offset = branch(pose_at_mouth(truth[truth_stamp]))
    converted_truth = {}
    for stamp, pose in truth.items():
        transformed = pose_at_mouth(pose)
        transformed[2] = wrap(transformed[2]+truth_offset)
        converted_truth[stamp] = transformed
    converted, offset = [], None
    for row in rows:
        result = dict(row)
        if row['initialized']:
            pose = pose_at_mouth(row['pose'])
            if offset is None:
                offset = branch(pose)
            # Direct Jacobian of (X*Exp(delta)).inverse()*[depth,0].
            # This equals inversePoseCovariance followed by openingFaceCovariance.
            jacobian = np.array([[-1.,0.,pose[1]], [0.,-1.,-pose[0]], [0.,0.,-1.]])
            covariance = np.array(row['covariance']).reshape(3,3)
            result['covariance'] = (jacobian@covariance@jacobian.T).ravel().tolist()
            pose[2] = wrap(pose[2]+offset)
            result['pose'] = pose.tolist()
            result['covariance_coordinates'] = 'additive'
        converted.append(result)
    return converted, converted_truth


def score_outputs(outputs, truth, warmup=1.):
    errors, nees, latencies, eigenvalues, processing_times = [], [], [], [], []
    valid = bool(outputs)
    modes = {}
    uninitialized = 0
    for row in outputs:
        modes[row['mode']] = modes.get(row['mode'], 0)+1
        latency = row.get('update_ms', 0.)
        if latency is not None and math.isfinite(latency):
            latencies.append(latency)
        processing_ms = row.get('processing_ms')
        if processing_ms is not None and math.isfinite(processing_ms):
            processing_times.append(processing_ms)
        if not row['initialized']:
            if row['stamp'] >= warmup:
                uninitialized += 1
            continue
        pose = np.array(row['pose'], dtype=float)
        covariance = np.array(row['covariance'], dtype=float).reshape(3, 3)
        finite = np.isfinite(pose).all() and np.isfinite(covariance).all()
        if not finite:
            valid = False
            continue
        eig = np.linalg.eigvalsh((covariance+covariance.T)/2)
        eigenvalues.append(float(eig[0]))
        valid &= bool(eig[0] >= -1e-10 and np.max(np.abs(covariance-covariance.T)) < 1e-8)
        if row['stamp'] < warmup:
            continue
        actual = truth[round(row['stamp'], 9)]
        delta = actual[:2]-pose[:2]
        angle = wrap(actual[2]-pose[2])
        errors.append((np.linalg.norm(delta), abs(angle)))
        c, s = math.cos(pose[2]), math.sin(pose[2])
        local = np.array([[c, s], [-s, c]]) @ delta
        a = math.sin(angle)/angle if abs(angle) > 1e-8 else 1.
        b = (1-math.cos(angle))/angle if abs(angle) > 1e-8 else 0.
        rho = np.array([[a, b], [-b, a]]) @ local / (a*a+b*b)
        tangent = np.r_[rho, angle]
        if row.get('covariance_coordinates') == 'additive':
            tangent = np.r_[delta, angle]
        if eig[0] > 0:
            nees.append(float(tangent @ np.linalg.solve(covariance, tangent)))
    error_array = np.array(errors) if errors else np.empty((0, 2))
    valid &= bool(errors) and not uninitialized
    return dict(valid=bool(valid), scored_samples=len(errors), uninitialized_after_warmup=uninitialized,
                translation_rmse_m=float(np.sqrt(np.mean(error_array[:, 0]**2))) if errors else None,
                yaw_rmse_deg=float(np.degrees(np.sqrt(np.mean(error_array[:, 1]**2)))) if errors else None,
                translation_p95_m=float(np.percentile(error_array[:, 0], 95)) if errors else None,
                translation_max_m=float(error_array[:, 0].max()) if errors else None,
                min_covariance_eigenvalue=min(eigenvalues) if eigenvalues else None,
                diagnostic_mean_nees=float(np.mean(nees)) if nees else None,
                latency_p95_ms=float(np.percentile(latencies, 95)) if latencies else None,
                latency_max_ms=max(latencies) if latencies else None,
                processing_p95_ms=float(np.percentile(processing_times, 95)) if processing_times else None,
                processing_max_ms=max(processing_times) if processing_times else None, modes=modes)


def accept_trial(kind, score, outputs):
    if kind == 'overlong':
        return bool(outputs) and all(not row['initialized'] and row['updates'] == 0 for row in outputs)
    if not score['valid']:
        return False
    if score['latency_p95_ms'] is None or score['latency_p95_ms'] > 50.:
        return False
    if score.get('processing_p95_ms') is not None and score['processing_p95_ms'] > 50.:
        return False
    if kind == 'noiseless':
        return score['translation_rmse_m'] < .001 and score['yaw_rmse_deg'] < .1
    if kind in ('nominal', 'clutter', 'delayed', 'swept'):
        return score['translation_rmse_m'] < .02 and score['yaw_rmse_deg'] < 1.
    return True  # Scenario-specific outage checks are evaluated separately below.


def failure_reasons(kind, score, outputs, checks):
    reasons = []
    if kind == 'overlong':
        if not accept_trial(kind, score, outputs):
            reasons.append('overlong marker was initialized or updated')
    else:
        if not score['valid']:
            reasons.append('invalid state/covariance or unavailable post-warmup output')
        if score['latency_p95_ms'] is None or score['latency_p95_ms'] > 50.:
            reasons.append('filter latency p95 unavailable or > 50 ms')
        if score.get('processing_p95_ms') is not None and score['processing_p95_ms'] > 50.:
            reasons.append('full scan processing p95 > 50 ms')
        if score['valid'] and kind in ('nominal', 'clutter', 'delayed', 'swept', 'noiseless'):
            translation, yaw = (.001, .1) if kind == 'noiseless' else (.02, 1.)
            if score['translation_rmse_m'] >= translation:
                reasons.append('translation RMSE >= {:g} mm'.format(translation*1000))
            if score['yaw_rmse_deg'] >= yaw:
                reasons.append('yaw RMSE >= {:g} degree'.format(yaw))
    reasons += ['scenario check: '+key for key, passed in checks.items() if not passed]
    return reasons


def scenario_checks(trial, outputs, inputs=""):
    rows = {round(row['stamp'], 9): row for row in outputs}
    checks = {}
    if trial.kind == 'delayed':
        checks['delayed_scans_replayed'] = max(row['replays'] for row in outputs) > 0
    if trial.kind == 'partial':
        checks['partial_mode_observed'] = any(row['mode'] == 'PARTIAL' for row in outputs if 2.2 <= row['stamp'] < 4.)
    if trial.kind in ('dropout', 'stale'):
        middle = [row for row in outputs if 2.3 <= row['stamp'] < 4.]
        checks['continuous_output'] = len(middle) >= 30
        checks['expected_mode'] = any(row['mode'] == ('ODOMETRY_STALE' if trial.kind == 'stale' else 'PROPAGATION_ONLY') for row in middle)
        if 2.3 in rows and 3.9 in rows:
            checks['covariance_increases'] = bool(np.trace(np.array(rows[3.9]['covariance']).reshape(3,3)) > np.trace(np.array(rows[2.3]['covariance']).reshape(3,3)))
        recovery = [row for row in outputs if 4. <= row['stamp'] <= 5.]
        checks['recovered_within_one_second'] = any(row['mode'] == 'TRACKING' and row['last_measurement'] >= 4. for row in recovery)
        if trial.kind == 'dropout':
            updates = {row['updates'] for row in outputs if 2. <= row['stamp'] < 4.}
            checks['no_outage_measurement_updates'] = len(updates) == 1
            outage = [row for row in outputs if 2. <= row['stamp'] < 4.]
            odom = [list(map(float, line.split()[1:])) for line in inputs.splitlines() if line.startswith('O ')]
            matches = bool(outage) and bool(odom)
            if matches:
                predicted = np.array(outage[0]['pose'])
                previous_stamp = outage[0]['stamp']
                for row in outage[1:]:
                    segments = sorted({previous_stamp, row['stamp']} | {sample[0] for sample in odom if previous_stamp < sample[0] < row['stamp']})
                    for left, right in zip(segments, segments[1:]):
                        sample = next((sample for sample in reversed(odom) if sample[0] <= left+1e-9), None)
                        if sample is None:
                            matches = False
                            break
                        predicted = advance(predicted, sample[1:], right-left)
                    matches &= bool(np.linalg.norm(predicted[:2]-np.array(row['pose'][:2])) < 1e-7 and abs(wrap(predicted[2]-row['pose'][2])) < 1e-7)
                    previous_stamp = row['stamp']
            checks['outage_mean_matches_odometry'] = matches
        if trial.kind == 'stale' and 2.3 in rows and 3.9 in rows:
            left, right = np.array(rows[2.3]['pose']), np.array(rows[3.9]['pose'])
            checks['stale_mean_holds'] = bool(np.linalg.norm(left[:2]-right[:2]) < 1e-8 and abs(wrap(left[2]-right[2])) < 1e-8)
    return checks


def fingerprints(binary):
    paths = list((SIMULATOR/'engine').rglob('*.py')) + list((SIMULATOR/'engine').rglob('*.c'))
    for suffix in ('*.hpp', '*.cpp', '*.py', '*.yaml'):
        for directory in ('include', 'src', 'tools', 'test', 'config'):
            paths += list((PACKAGE/directory).rglob(suffix))
    paths += [PACKAGE/'CMakeLists.txt', binary]
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sorted(set(paths)) if path.is_file()}
    revision = subprocess.run(['git', '-C', str(PACKAGE), 'rev-parse', 'HEAD'], text=True, capture_output=True)
    return dict(files_sha256=hashes, git_head=revision.stdout.strip(), python=platform.python_version(),
                numpy=np.__version__, platform=platform.platform(), machine=platform.machine())


def save_trial(directory, inputs, truth, metadata, outputs, baseline, baseline_stdout):
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'input.txt').write_text(inputs)
    (directory/'config.json').write_text(json.dumps(metadata, indent=2)+'\n')
    (directory/'output.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in outputs))
    (directory/'baseline.jsonl').write_text(baseline_stdout)
    baseline_by_stamp = {row['stamp']: row for row in baseline}
    with (directory/'trajectory.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['stamp', 'true_x', 'true_y', 'true_yaw', 'x', 'y', 'yaw', 'baseline_x', 'baseline_y', 'baseline_yaw', 'initialized', 'mode', 'translation_error_m', 'yaw_error_deg', 'covariance_trace', 'nis', 'dof', 'update_ms'])
        for row in outputs:
            actual = truth[round(row['stamp'], 9)]
            base = baseline_by_stamp.get(row['stamp'], {}).get('pose', [None]*3)
            writer.writerow([row['stamp'], *actual, *row['pose'], *base, row['initialized'], row['mode'],
                             np.linalg.norm(actual[:2]-np.array(row['pose'][:2])) if row['initialized'] else None,
                             math.degrees(wrap(actual[2]-row['pose'][2])) if row['initialized'] else None,
                             np.trace(np.array(row['covariance']).reshape(3,3)), row['nis'], row['dof'], row['update_ms']])


def save_plot(destination, summaries):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    good = [row for row in summaries if 'score' in row and row['score']['translation_rmse_m'] is not None]
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    positions = np.arange(len(good))
    axes[0].plot(positions, [row['score']['translation_rmse_m']*1000 for row in good], '.', label='internal robot-in-marker state')
    axes[0].plot(positions, [row['baseline_score']['translation_rmse_m']*1000 for row in good], '.', label='same-posterior odometry only (internal state)')
    axes[0].plot(positions, [row.get('docking_score', row['score'])['translation_rmse_m']*1000 for row in good], '.', label='requested mouth pose in AMR base')
    axes[0].axhline(20, color='red', linestyle='--', label='nominal target')
    axes[0].set_ylabel('translation RMSE (mm)')
    axes[0].legend()
    axes[1].plot(positions, [row.get('docking_score', row['score'])['yaw_rmse_deg'] for row in good], '.')
    axes[1].axhline(1, color='red', linestyle='--')
    axes[1].set_ylabel('yaw RMSE (degrees)')
    axes[1].set_xticks(positions)
    axes[1].set_xticklabels([row['name'] for row in good], rotation=90, fontsize=6)
    fig.tight_layout()
    fig.savefig(destination, dpi=140)
    plt.close(fig)


def distance_summary(results, score_key='score', pass_key='passed'):
    summaries = []
    distances = sorted({row.get('distance') for row in results if row['kind'] == 'nominal' and row.get('distance') is not None})
    for distance in distances:
        rows = [row for row in results if row['kind'] == 'nominal' and row.get('distance') == distance]
        scored = [row[score_key] for row in rows if row.get(score_key, {}).get('translation_rmse_m') is not None]
        samples = sum(score['scored_samples'] for score in scored)
        summary = dict(initial_lidar_range_m=distance, trials=len(rows), passed=sum(row[pass_key] for row in rows),
                       acquired=sum(row.get('acquired', False) for row in rows),
                       acquired_within_one_second=sum(row.get('acquired_within_one_second', False) for row in rows),
                       translation_rmse_m=math.sqrt(sum(score['translation_rmse_m']**2*score['scored_samples'] for score in scored)/samples) if samples else None,
                       yaw_rmse_deg=math.sqrt(sum(score['yaw_rmse_deg']**2*score['scored_samples'] for score in scored)/samples) if samples else None,
                       worst_translation_rmse_m=max((score['translation_rmse_m'] for score in scored), default=None),
                       worst_yaw_rmse_deg=max((score['yaw_rmse_deg'] for score in scored), default=None))
        summaries.append(summary)
    return summaries


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('legacy','mir_v150'), default='legacy')
    parser.add_argument('--replay', type=Path, default=PACKAGE/'build'/'vshape_replay')
    parser.add_argument('--output', type=Path, default=None)
    parser.add_argument('--seeds', type=int, default=30)
    parser.add_argument('--adverse-seeds', type=int, default=3)
    parser.add_argument('--duration', type=float, default=6.)
    parser.add_argument('--rays', type=int, default=None)
    parser.add_argument('--kind', choices=('nominal','dropout','partial','clutter','delayed','stale','swept','noiseless','overlong'), action='append')
    args = parser.parse_args(argv)
    if args.seeds < 0 or args.adverse_seeds < 0 or args.duration < 1.5 or (args.rays is not None and args.rays < 100):
        parser.error('nonnegative seeds, duration >= 1.5 and rays >= 100 required')
    if not args.replay.is_file():
        parser.error('production replay executable missing: {}'.format(args.replay))
    if args.output is None:
        args.output = PACKAGE/'reports'/('mir_v150' if args.profile == 'mir_v150' else 'simple_sim')
    args.output.mkdir(parents=True, exist_ok=True)
    provenance = fingerprints(args.replay)
    provenance['arguments'] = {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()}
    (args.output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    summaries = []
    trials = [trial for trial in make_trials(args.seeds, args.adverse_seeds, args.profile) if not args.kind or trial.kind in args.kind]
    for trial in trials:
        directory = args.output/trial.name
        directory.mkdir(parents=True, exist_ok=True)
        summary = dict(name=trial.name, **asdict(trial), passed=False, docking_passed=False)
        try:
            inputs, truth, metadata = generate_trial(trial, args.duration, args.rays, args.profile)
            (directory/'input.txt').write_text(inputs)
            (directory/'config.json').write_text(json.dumps(metadata, indent=2)+'\n')
            outputs, stdout, elapsed = replay(args.replay, inputs, directory/'output.jsonl')
            first_initialized = next((row for row in outputs if row['initialized']), None)
            summary.update(acquired=first_initialized is not None, acquisition_stamp=first_initialized['stamp'] if first_initialized else None, acquired_within_one_second=first_initialized is not None and first_initialized['stamp'] <= 1.)
            if trial.kind == 'overlong' or first_initialized is None:
                baseline, baseline_stdout = [], ''
                summary['baseline_status'] = 'No initialized posterior; baseline unavailable (expected only for overlong rejection).'
            else:
                baseline_input = baseline_inputs(inputs, outputs)
                (directory/'baseline_input.txt').write_text(baseline_input)
                baseline, baseline_stdout, _ = replay(args.replay, baseline_input, directory/'baseline.jsonl')
            save_trial(directory, inputs, truth, metadata, outputs, baseline, baseline_stdout)
            score = score_outputs(outputs, truth)
            baseline_score = score_outputs(baseline, truth)
            depth = metadata['config'].get('arm_length', .5)*math.cos(math.radians(metadata['config'].get('opening_angle_deg', 70.))/2)
            docking_outputs, docking_truth = output_to_mouth(outputs, truth, depth)
            docking_score = score_outputs(docking_outputs, docking_truth)
            (directory/'docking_output.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in docking_outputs))
            checks = scenario_checks(trial, outputs, inputs)
            if trial.kind == 'delayed':
                chronological_input = chronological_inputs(inputs)
                (directory/'chronological_input.txt').write_text(chronological_input)
                reference, _, _ = replay(args.replay, chronological_input, directory/'chronological.jsonl')
                checks['replay_matches_chronological_pose'] = bool(np.max(np.abs(np.array(outputs[-1]['pose'])-reference[-1]['pose'])) < 1e-8)
                checks['replay_matches_chronological_covariance'] = bool(np.max(np.abs(np.array(outputs[-1]['covariance'])-reference[-1]['covariance'])) < 1e-8)
            summary.update(score=score, baseline_score=baseline_score, checks=checks,
                           docking_score=docking_score,
                           docking_passed=accept_trial(trial.kind, docking_score, docking_outputs) and all(checks.values()),
                           docking_failure_reasons=failure_reasons(trial.kind, docking_score, docking_outputs, checks),
                           failure_reasons=failure_reasons(trial.kind, score, outputs, checks),
                           replay_wall_seconds=elapsed,
                           raw_scan_count=sum(line.startswith('S ') for line in inputs.splitlines()),
                           amortized_replay_ms_per_scan=1000*elapsed/sum(line.startswith('S ') for line in inputs.splitlines()),
                           passed=accept_trial(trial.kind, score, outputs) and all(checks.values()))
        except Exception as error:
            summary['error'] = '{}: {}'.format(type(error).__name__, error)
        summaries.append(summary)
        print('{} {}{}'.format('PASS' if summary['passed'] else 'FAIL', trial.name,
                               ': '+summary['error'] if 'error' in summary else ''), flush=True)
    failed = sum(not row['passed'] for row in summaries)
    range_summary = distance_summary(summaries)
    docking_range_summary = distance_summary(summaries, 'docking_score', 'docking_passed')
    docking_failed = sum(not row.get('docking_passed', False) for row in summaries)
    result = dict(profile=args.profile, distance_summary=range_summary,
                  docking_distance_summary=docking_range_summary, docking_failures=docking_failed, trials=len(trials), completed=len(summaries), failures=failed, dropped_seeds=0,
                  nominal_seeds=sum(row['kind']=='nominal' for row in summaries),
                  formal_gaussian_consistency='Separate fixed-association C++ experiment; these NEES values are diagnostic only.',
                  targets=dict(translation_rmse_m=.02, yaw_rmse_deg=1., latency_p95_ms=50.), results=summaries)
    (args.output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    with (args.output/'summary.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['name','passed','translation_rmse_m','yaw_rmse_deg','baseline_translation_rmse_m','baseline_yaw_rmse_deg','filter_update_p95_ms','full_scan_processing_p95_ms','raw_scan_count','cli_wall_seconds','amortized_cli_wall_ms_per_scan','error','failure_reasons','docking_passed','docking_translation_rmse_m','docking_yaw_rmse_deg','docking_failure_reasons'])
        for row in summaries:
            score, baseline = row.get('score', {}), row.get('baseline_score', {})
            writer.writerow([row['name'], row['passed'], score.get('translation_rmse_m'), score.get('yaw_rmse_deg'), baseline.get('translation_rmse_m'), baseline.get('yaw_rmse_deg'), score.get('latency_p95_ms'), score.get('processing_p95_ms'), row.get('raw_scan_count'), row.get('replay_wall_seconds'), row.get('amortized_replay_ms_per_scan'), row.get('error',''), '; '.join(row.get('failure_reasons', [])), row.get('docking_passed', False), row.get('docking_score', {}).get('translation_rmse_m'), row.get('docking_score', {}).get('yaw_rmse_deg'), '; '.join(row.get('docking_failure_reasons', []))])
    save_plot(args.output/'errors.png', summaries)
    lines = ['# Real simple-sim raw-scan validation', '',
             '{} trials, {} requested docking-pose failures; {} internal core-state failures retained. 0 dropped seeds. {} nominal independent seeds.'.format(len(trials), docking_failed, failed, result['nominal_seeds']), '',
             ('MiR V150 profile: 0.150 m arms, 120 degree opening, 259.8 mm width, 75 mm depth, 100 mm physical height at z=0.213–0.313 m. Scanner at z=0.263 m with actual base mount (0.362, -0.272, -45 degrees), 1651 rays across 270 degrees (0.163636 degree spacing), independent 10 mm range noise. Distances are initial LiDAR-to-apex ranges; robot placement accounts for the nonzero mount. Motion speeds are one-quarter of the legacy profile.' if args.profile == 'mir_v150' else 'Legacy profile: 0.5 m arms, 70 degree opening and 1441 rays across 180 degrees.'), '',
             'The overlong-marker adversarial case ray-casts arms twice the configured target length and requires every output to stay uninitialized with zero measurement updates.', '',
             'Requested docking output: opening-face midpoint in the AMR base frame with the outward heading branch fixed at acquisition. Depth is configured arm length times cos(half opening angle), 75 mm for MiR V150. Its additive covariance follows the same inverse-pose and midpoint-shift Jacobians as the ROS node. Internal core scores separately measure robot-in-apex-marker coordinates and retain the original pass/fail results. These translation errors differ because inversion couples position and heading.', '',
             'Targets: nominal post-1-second translation RMSE <20 mm, yaw RMSE <1 degree; update latency p95 <50 ms. Noiseless local-prior target: <1 mm / 0.1 degree.', '',
             'The runner exercises 20 Hz scans with at most 24 selected points per arm, a 1 second replay history, and a 5000 event cap. Filter-update latency is the production update_ms diagnostic. Full scan processing latency is processing_ms around Tracker::addScan, including conversion, detection, update and replay; amortized wall time of the complete CLI subprocess per raw scan is also retained in summary.json. It includes Python subprocess overhead and is an estimate on this machine, not a hard real-time bound.', '',
             'The odometry-only baseline starts at the first initialized tracker posterior and receives the same twist samples. Truth never enters tracking updates. Front/rear/side use rotated sensor extrinsics. Swept scans use straight motion matching the scanner first-ray convention.', '',
             'Full scans, output JSON, baseline inputs, truth CSV, exact configurations, seeds and source SHA256 fingerprints are retained. Diagnostic NEES is conditional on association; formal fixed-association Gaussian consistency is a separate C++ experiment.', '',
             '| Trial | Docking pass | Docking mm | Docking yaw deg | Core pass | Core mm | Core yaw deg | Baseline core mm | Filter p95 ms | Full scan p95 ms |', '|---|---|---:|---:|---|---:|---:|---:|---:|---:|']
    if docking_range_summary:
        detailed_header = lines[-2:]
        lines = lines[:-2]
        lines += ['Requested outward opening-face pose in the AMR base frame. All samples and criteria are retained; these synthetic results do not establish a physical-sensor reliability guarantee.', '',
                  '| Initial range m | Docking passed / trials | Pooled translation mm | Pooled yaw deg | Worst translation mm | Worst yaw deg |', '|---:|---:|---:|---:|---:|---:|']
        for item in docking_range_summary:
            def number(value, scale=1):
                return '{:.3f}'.format(value*scale) if value is not None else '—'
            lines.append('| {} | {} / {} | {} | {} | {} | {} |'.format(item['initial_lidar_range_m'], item['passed'], item['trials'], number(item['translation_rmse_m'],1000), number(item['yaw_rmse_deg']), number(item['worst_translation_rmse_m'],1000), number(item['worst_yaw_rmse_deg'])))
        lines += ['', 'Internal robot-in-apex-marker coordinate results follow; the original failures are preserved.', ''] + detailed_header
    if range_summary:
        detailed_header = lines[-2:]
        lines = lines[:-2]
        lines += ['Acquisition and accuracy by initial LiDAR range (all seeds retained; five different motion cases per range in the default 30-seed matrix). These synthetic samples do not establish a physical-sensor reliability guarantee.', '',
                  '| Initial range m | Acquired / trials | Within 1 s | Passed | Pooled translation mm | Pooled yaw deg | Worst translation mm | Worst yaw deg |', '|---:|---:|---:|---:|---:|---:|---:|---:|']
        for item in range_summary:
            def number(value, scale=1):
                return '{:.3f}'.format(value*scale) if value is not None else '—'
            lines.append('| {} | {} / {} | {} | {} / {} | {} | {} | {} | {} |'.format(item['initial_lidar_range_m'], item['acquired'], item['trials'], item['acquired_within_one_second'], item['passed'], item['trials'], number(item['translation_rmse_m'],1000), number(item['yaw_rmse_deg']), number(item['worst_translation_rmse_m'],1000), number(item['worst_yaw_rmse_deg'])))
        failing_distances = [str(item['initial_lidar_range_m']) for item in range_summary if item['passed'] != item['trials']]
        lines += ['', 'At least one nominal case failed the unchanged accuracy targets at initial ranges: {} m. Acquisition success is reported separately from accuracy; a detected marker is not an accuracy guarantee.'.format(', '.join(failing_distances) if failing_distances else 'none'), ''] + detailed_header
    diagnostics = []
    for row in summaries:
        score, baseline = row.get('score', {}), row.get('baseline_score', {})
        def display(value, scale=1):
            return '{:.3f}'.format(value*scale) if value is not None else '—'
        lines.append('| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |'.format(row['name'], row.get('docking_passed', False), display(row.get('docking_score', {}).get('translation_rmse_m'),1000), display(row.get('docking_score', {}).get('yaw_rmse_deg')), row['passed'], display(score.get('translation_rmse_m'),1000), display(score.get('yaw_rmse_deg')), display(baseline.get('translation_rmse_m'),1000), display(score.get('latency_p95_ms')), display(score.get('processing_p95_ms'))))
        if 'error' in row:
            diagnostics.append('{}: {}'.format(row['name'], row['error']))
        if row.get('docking_failure_reasons'):
            diagnostics.append('{} docking output: {}'.format(row['name'], '; '.join(row['docking_failure_reasons'])))
        if row.get('failure_reasons'):
            diagnostics.append('{}: {}'.format(row['name'], '; '.join(row['failure_reasons'])))
    lines += [''] + [item+'\n' for item in diagnostics] + ['![Errors](errors.png)', '']
    (args.output/'report.md').write_text('\n'.join(lines))
    print('Report: {}'.format(args.output/'report.md'))
    return 1 if failed or not trials else 0


if __name__ == '__main__':
    sys.exit(main())
