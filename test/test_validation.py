"""Validation harness contracts; run with python3 -m unittest discover -s test -p test_validation.py."""
import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np

PATH = Path(__file__).resolve().parents[1] / 'tools' / 'validate_simple_sim.py'
spec = importlib.util.spec_from_file_location('validate_simple_sim', PATH)
validation = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = validation
spec.loader.exec_module(validation)


class ValidationTests(unittest.TestCase):
    def test_seed_schedule_covers_all_views_and_motions_without_dropped_seeds(self):
        trials = validation.make_trials(30, 1)
        nominal = [t for t in trials if t.kind == 'nominal']
        self.assertEqual(len(nominal), 30)
        self.assertEqual(len({t.seed for t in nominal}), 30)
        self.assertEqual(len({(t.view, t.motion) for t in nominal}), 15)
        self.assertTrue({'dropout', 'partial', 'clutter', 'delayed', 'stale', 'swept', 'noiseless'} <= {t.kind for t in trials})

    def test_se2_truth_propagation_matches_turn_and_lateral_motion(self):
        pose = validation.advance(np.zeros(3), np.array([1., 0., 1.]), math.pi / 2)
        np.testing.assert_allclose(pose, [1., 1., math.pi / 2], atol=1e-12)
        np.testing.assert_allclose(validation.advance([0, 0, math.pi/2], [0, 1, 0], 1), [-1, 0, math.pi/2], atol=1e-12)

    def test_scoring_uses_truth_and_wraps_yaw_excludes_warmup(self):
        outputs = [dict(stamp=t, initialized=True, pose=[.01, 0, -math.pi+.01], covariance=np.eye(3).ravel().tolist(), update_ms=2, processing_ms=3, mode='TRACKING') for t in [0., 2., 3.]]
        outputs[0]['pose'][0] = 100.
        truth = {t: np.array([0., 0., math.pi]) for t in [0., 2., 3.]}
        score = validation.score_outputs(outputs, truth, warmup=1.)
        self.assertAlmostEqual(score['translation_rmse_m'], .01)
        self.assertAlmostEqual(score['yaw_rmse_deg'], math.degrees(.01))
        self.assertEqual(score['scored_samples'], 2)
        self.assertEqual(score['latency_p95_ms'], 2)
        self.assertEqual(score['processing_p95_ms'], 3)
        self.assertEqual(score['processing_max_ms'], 3)

    def test_bad_covariance_and_uninitialized_samples_fail_acceptance(self):
        outputs = [dict(stamp=2., initialized=False, pose=[0, 0, 0], covariance=(-np.eye(3)).ravel().tolist(), update_ms=0, mode='UNINITIALIZED')]
        score = validation.score_outputs(outputs, {2.: np.zeros(3)}, warmup=1.)
        self.assertFalse(score['valid'])
        self.assertFalse(validation.accept_trial('nominal', score, outputs))

    def test_parser_rejects_missing_or_extra_output(self):
        row = dict(stamp=0., initialized=True, pose=[0, 0, 0], covariance=np.eye(3).ravel().tolist(), mode='TRACKING', reason='', last_measurement=0., odom_age=0., nis=0., dof=10, points=[5, 5], eigenvalues=[1,2,3], updates=1, rejected=0, replays=0, update_ms=.1)
        import json
        parsed = validation.parse_outputs(json.dumps(row)+'\n', [0.])
        self.assertEqual(len(parsed), 1)
        with self.assertRaises(ValueError):
            validation.parse_outputs('', [0.])
        with self.assertRaises(ValueError):
            validation.parse_outputs(json.dumps(row)+'\n', [1.])

    def test_real_scanner_ranges_use_configured_sensor_and_no_labels_in_protocol(self):
        trial = validation.Trial('nominal', 'stationary', 'rear', 13)
        inputs, truth, metadata = validation.generate_trial(trial, duration=.2, n_rays=721)
        scans = [line.split() for line in inputs.splitlines() if line.startswith('S ')]
        self.assertEqual(len(scans), 5)
        self.assertEqual(metadata['sensor']['scan_hz'], 20.)
        self.assertEqual(len(scans[0]), 6+int(scans[0][5]))
        self.assertGreater(sum(math.isfinite(float(x)) for x in scans[0][6:]), 15)
        self.assertEqual(float(scans[0][4]), 0.)
        self.assertFalse(any(line.startswith('I ') for line in inputs.splitlines()))
        self.assertTrue(all(line[0] in 'COSP' for line in inputs.splitlines()))
        self.assertEqual(metadata['scanner'], 'engine.sim.scanner.Scanner')
        self.assertEqual(len(truth), 5)

    def test_baseline_starts_at_same_posterior_and_uses_only_twist(self):
        inputs = 'C lidar_yaw 0\nO 0 .1 0 0\nS 0 0 .1 0 2 1 1\nP 0\nO .1 .1 0 0\nP .1\n'
        outputs = [dict(stamp=0., initialized=True, pose=[2., 0., 0.], covariance=np.eye(3).ravel().tolist())]
        baseline = validation.baseline_inputs(inputs, outputs)
        self.assertIn('I 0.000000000 2 0 0', baseline)
        self.assertNotIn('S ', baseline)
        self.assertIn('O 0.000000000 .1 0 0', baseline)
        self.assertIn('P .1', baseline)

    def test_dropout_check_detects_mean_that_does_not_follow_same_odometry(self):
        trial = validation.Trial('dropout', 'approach', 'front', 2)
        rows = [dict(stamp=round(2.+i*.05, 9), initialized=True, pose=[0., 0., 0.], covariance=(np.eye(3)*(1+i)).ravel().tolist(), mode='PROPAGATION_ONLY', last_measurement=1.9, updates=1) for i in range(41)]
        rows[-1].update(mode='TRACKING', last_measurement=4., updates=2)
        checks = validation.scenario_checks(trial, rows, 'O 2 .1 0 0\n')
        self.assertFalse(checks['outage_mean_matches_odometry'])

    def test_mouth_output_matches_inverse_shift_and_outward_branch(self):
        rows = [dict(stamp=0., initialized=True, pose=[2., 0., math.pi], covariance=np.eye(3).ravel().tolist()),
                dict(stamp=1., initialized=True, pose=[2., 0., math.pi+2.], covariance=np.eye(3).ravel().tolist())]
        truth = {row['stamp']: np.array(row['pose']) for row in rows}
        converted, converted_truth = validation.output_to_mouth(rows, truth, depth=.075)
        np.testing.assert_allclose(converted[0]['pose'], [1.925, 0., 0.], atol=1e-12)
        self.assertAlmostEqual(converted[1]['pose'][2], -2.)
        self.assertEqual(converted[0]['covariance_coordinates'], 'additive')
        np.testing.assert_allclose(converted_truth[0.], converted[0]['pose'], atol=1e-12)
        self.assertEqual(rows[0]['pose'], [2., 0., math.pi])

    def test_mouth_covariance_matches_right_perturbation_finite_difference(self):
        pose = np.array([1.2, -.4, 2.8])
        covariance = np.array([[.03,.004,.001],[.004,.02,-.002],[.001,-.002,.006]])
        def converted(value):
            row = dict(stamp=0., initialized=True, pose=value.tolist(), covariance=covariance.ravel().tolist())
            return validation.output_to_mouth([row], {0.: pose}, depth=.075)[0][0]
        result = converted(pose)
        jacobian = np.zeros((3,3))
        for axis in range(3):
            perturbation = np.zeros(3); perturbation[axis] = 1e-6
            plus = np.array(converted(validation.advance(pose, perturbation, 1.))['pose'])
            minus = np.array(converted(validation.advance(pose, -perturbation, 1.))['pose'])
            delta = plus-minus; delta[2] = validation.wrap(delta[2])
            jacobian[:,axis] = delta/2e-6
        np.testing.assert_allclose(np.array(result['covariance']).reshape(3,3), jacobian@covariance@jacobian.T, atol=1e-9)

    def test_additive_output_nees_uses_additive_coordinate_error(self):
        row = dict(stamp=2., initialized=True, pose=[.01,0.,math.pi/2], covariance=np.diag([.0001,1.,1.]).ravel().tolist(), covariance_coordinates='additive', mode='TRACKING', update_ms=0.)
        score = validation.score_outputs([row], {2.: np.array([0.,0.,math.pi/2])})
        self.assertAlmostEqual(score['diagnostic_mean_nees'], 1.)

    def test_small_profile_repeats_each_motion_range_with_independent_seeds(self):
        from collections import Counter
        trials = validation.make_trials(150, 3, profile='mir_v150')
        nominal = [trial for trial in trials if trial.kind == 'nominal']
        self.assertEqual(len(nominal), 150)
        self.assertEqual(len({trial.seed for trial in nominal}), 150)
        self.assertEqual(set(Counter((trial.motion, trial.distance) for trial in nominal).values()), {5})
        self.assertEqual(len(trials)-len(nominal), 24)

    def test_failed_accuracy_and_scenario_checks_are_reported_without_relaxing_targets(self):
        score = dict(valid=True, translation_rmse_m=.02, yaw_rmse_deg=1., latency_p95_ms=1., processing_p95_ms=2.)
        self.assertFalse(validation.accept_trial('nominal', score, []))
        reasons = validation.failure_reasons('nominal', score, [], {'recovered_within_one_second': False})
        self.assertEqual(reasons, ['translation RMSE >= 20 mm', 'yaw RMSE >= 1 degree', 'scenario check: recovered_within_one_second'])
        self.assertEqual(validation.failure_reasons('overlong', {'valid': False}, [dict(initialized=False, updates=0)], {}), [])

    def test_small_profile_covers_requested_ranges_and_actual_mount(self):
        trials = validation.make_trials(30, 0, profile='mir_v150')
        self.assertEqual(len(trials), 30)
        self.assertEqual({trial.distance for trial in trials}, {.7, 1., 1.5, 2., 2.5, 3.})
        self.assertEqual(len({trial.seed for trial in trials}), 30)
        inputs, _, metadata = validation.generate_trial(trials[0], duration=.2, profile='mir_v150')
        self.assertEqual(metadata['sensor']['n_rays'], 1651)
        self.assertEqual(metadata['sensor']['fov_deg'], 270.)
        self.assertEqual(metadata['sensor']['x'], .362)
        self.assertEqual(metadata['sensor']['y'], -.272)
        self.assertAlmostEqual(metadata['sensor']['yaw'], -math.pi/4)
        self.assertEqual(metadata['scene'][0]['arm_length'], .15)
        self.assertEqual(metadata['scene'][0]['opening_angle_deg'], 120.)
        self.assertAlmostEqual(metadata['initial_lidar_range_m'], .7)
        self.assertEqual(metadata['config']['min_support'], .045)
        self.assertFalse(any(line.startswith('I ') for line in inputs.splitlines()))

    def test_distance_summary_counts_failed_seeds_and_acquisition(self):
        results = [dict(kind='nominal', distance=2., passed=passed, acquired=True, acquired_within_one_second=True,
                        score=dict(scored_samples=2, translation_rmse_m=error, yaw_rmse_deg=.5)) for passed,error in [(True,.01),(False,.03)]]
        aggregate = validation.distance_summary(results)
        self.assertEqual(aggregate[0]['trials'], 2)
        self.assertEqual(aggregate[0]['passed'], 1)
        self.assertEqual(aggregate[0]['acquired'], 2)
        self.assertAlmostEqual(aggregate[0]['translation_rmse_m'], math.sqrt(.0005))
        for row in results:
            row.update(docking_passed=True, docking_score=dict(row['score'], translation_rmse_m=.003))
        docking = validation.distance_summary(results, 'docking_score', 'docking_passed')
        self.assertEqual(docking[0]['passed'], 2)
        self.assertAlmostEqual(docking[0]['translation_rmse_m'], .003)
        self.assertEqual(aggregate[0]['passed'], 1)

    def test_overlong_real_v_is_a_deliberate_rejection_case(self):
        trial = validation.Trial('overlong', 'stationary', 'front', 2000)
        _, _, metadata = validation.generate_trial(trial, duration=.2, n_rays=721)
        self.assertEqual(metadata['scene'][0]['arm_length'], 1.)
        row = dict(initialized=False, updates=0)
        self.assertTrue(validation.accept_trial('overlong', dict(valid=False), [row]))
        self.assertFalse(validation.accept_trial('overlong', dict(valid=False), [dict(initialized=True, updates=0)]))
        self.assertFalse(validation.accept_trial('overlong', dict(valid=False), []))

    def test_scenario_checks_are_json_serializable(self):
        import json
        trial = validation.Trial('stale', 'approach', 'front', 2)
        rows = [dict(stamp=round(2.+i*.05, 9), initialized=True, pose=[0., 0., 0.], covariance=(np.eye(3)*(1+i)).ravel().tolist(), mode='ODOMETRY_STALE', last_measurement=1.9, updates=1) for i in range(41)]
        rows[-1].update(mode='TRACKING', last_measurement=4., updates=2)
        self.assertIsInstance(json.dumps(validation.scenario_checks(trial, rows)), str)

    def test_chronological_reference_keeps_identical_raw_scans(self):
        inputs = 'C range_sigma .01\nO 0 .1 0 0\nP 0\nO .1 .1 0 0\nS 0 -.1 .01 0 2 1 2\nP .1\n'
        reference = validation.chronological_inputs(inputs)
        self.assertLess(reference.index('S 0'), reference.index('P 0'))
        self.assertEqual([line for line in reference.splitlines() if line.startswith('S ')], [line for line in inputs.splitlines() if line.startswith('S ')])

    def test_swept_scan_is_straight_and_has_beam_timing(self):
        trial = validation.Trial('swept', 'approach', 'front', 1)
        inputs, _, metadata = validation.generate_trial(trial, duration=.2, n_rays=721)
        scan = next(line.split() for line in inputs.splitlines() if line.startswith('S '))
        self.assertGreater(float(scan[4]), 0.)
        self.assertEqual(metadata['truth_twist'][1:], [0., 0.])


if __name__ == '__main__':
    unittest.main()
