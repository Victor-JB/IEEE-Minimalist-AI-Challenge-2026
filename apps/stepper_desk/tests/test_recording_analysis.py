import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest

MODULE = Path(__file__).resolve().parents[1] / 'scripts' / 'analyze-recording.py'
spec = importlib.util.spec_from_file_location('analyze_recording', MODULE)
analyzer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analyzer)


def sample(time, mid=0, state=0, count=0, angle=0, run=0, error=0):
    axes = [dict(angle=0, reference=0, target=0, count=0, direction=0,
                 age=10, magnet=0x20, sensor_error=0) for _ in range(3)]
    axes[0].update(count=count, angle=angle)
    return dict(type='sample', version=1, time=time,
                sample=dict(v=1, error=error, owner=5, state=state, id=mid,
                            ms=int(time*1000), run=run, axes=axes))


def trial(mid=1, direction=0, start=0):
    sign = -1 if direction == 0 else 1
    events = [sample(start, mid=mid-1),
              dict(type='command', version=1, time=start+.1, text=f'JOG {mid} 0 {direction} 1')]
    for i in range(21):
        events.append(sample(start+.2+i*.1, mid, 3, i*30, sign*i*300, 1))
    events.extend([sample(start+2.3, mid, 5, 600, sign*6000),
                   sample(start+2.7, mid, 5, 600, sign*6000),
                   sample(start+2.8, mid, 5, 600, sign*6000)])
    return events


class AnalysisTests(unittest.TestCase):
    def test_clean_trial_and_regression(self):
        report = analyzer.analyze(trial())
        move = report['moves'][0]
        self.assertTrue(move['accepted'])
        self.assertTrue(move['ratio_candidate'])
        self.assertAlmostEqual(move['pulses_per_degree'], 100)
        self.assertAlmostEqual(move['interior_regression']['elbow']['pulses_per_degree'], 100)
        self.assertFalse(report['ratio_consistency']['elbow']['consistency_criteria_met'])

    def test_explicit_rate_command(self):
        events=trial()
        events[1]['text']='JOG 1 0 0 100 1'
        move=analyzer.analyze(events)['moves'][0]
        self.assertTrue(move['ratio_candidate'])
        self.assertEqual(move['nominal_rate_hz'],100)
        events[1]['text']='JOG 1 0 0 2401 1'
        self.assertFalse(analyzer.analyze(events)['moves'][0]['accepted'])

    def test_three_both_directions_still_does_not_verify(self):
        events = trial()
        for mid, direction, start in [(2, 1, 3), (3, 0, 6)]:
            segment = trial(mid, direction, start)
            base_angle = events[-1]['sample']['axes'][0]['angle']
            segment[0] = copy.deepcopy(events[-1])
            segment[0]['time'] = start
            segment[0]['sample']['ms'] = int(start*1000)
            for event in segment[1:]:
                if event['type'] == 'sample':
                    event['sample']['axes'][0]['angle'] += base_angle
            events += segment
        ratio = analyzer.analyze(events)['ratio_consistency']['elbow']
        self.assertTrue(ratio['consistency_criteria_met'])
        self.assertFalse(ratio['verified'])

    def test_command_without_acceptance_is_not_motion(self):
        events = trial()[:2] + [sample(.2, error=-22)]
        move = analyzer.analyze(events)['moves'][0]
        self.assertFalse(move['accepted'])
        self.assertEqual(move['response_errors'], [-22])

    def test_idle_and_existing_id_not_accepted(self):
        self.assertEqual(analyzer.analyze([sample(0)])['moves'], [])
        events = trial()
        events[0]['sample']['id'] = 1
        self.assertFalse(analyzer.analyze(events)['moves'][0]['accepted'])

    def test_fault_and_missing_settlement_reject(self):
        events = trial()
        events[-3]['sample']['state'] = 7
        self.assertFalse(analyzer.analyze(events)['moves'][0]['ratio_candidate'])
        self.assertEqual(analyzer.analyze(events)['moves'][0]['fault_reason'], 'sensor_unavailable')
        self.assertFalse(analyzer.analyze(trial()[:-2])['moves'][0]['ratio_candidate'])

    def test_zero_response_and_too_few_pulses(self):
        events = trial()
        for e in events:
            if e['type'] == 'sample':
                e['sample']['axes'][0]['angle'] = 0
                e['sample']['axes'][0]['count'] = min(25, e['sample']['axes'][0]['count'])
        move = analyzer.analyze(events)['moves'][0]
        self.assertIsNone(move['pulses_per_degree'])
        self.assertFalse(move['ratio_candidate'])
        self.assertIn('no resolved net angle response', move['ratio_rejection_reasons'])

    def test_later_idle_drift_not_used_as_endpoint(self):
        events = trial() + [sample(30, 1, 5, 600, -12000)]
        self.assertEqual(analyzer.analyze(events)['moves'][0]['final_angles_deg']['elbow'], -6)

    def test_settlement_and_untouched_drift_rejected(self):
        events=trial();events[-1]['sample']['axes'][0]['angle']-=200
        move=analyzer.analyze(events)['moves'][0]
        self.assertFalse(move['ratio_candidate'])
        self.assertIn('unstable settled endpoint',move['ratio_rejection_reasons'])
        events=trial();events[8]['sample']['axes'][1]['angle']=900
        self.assertIn('uncommanded joint moved',analyzer.analyze(events)['moves'][0]['ratio_rejection_reasons'])
        events=trial()
        for e in events[-2:]:e['sample']['axes'][0]['angle']-=400
        self.assertIn('excessive post-stop drift',analyzer.analyze(events)['moves'][0]['ratio_rejection_reasons'])

    def test_powered_settle_uses_awake_endpoints(self):
        events=self.powered_trial()
        move=analyzer.analyze(events)['moves'][0]
        self.assertTrue(move['ratio_candidate'],move['ratio_rejection_reasons'])
        self.assertEqual(move['measurement_mode'],'powered_settle')
        self.assertAlmostEqual(move['pulses_per_degree'],100)
        self.assertAlmostEqual(move['sleep_drift_deg']['elbow'],-1)
        self.assertEqual(move['asleep_final_angles_deg']['elbow'],-7)

    def powered_trial(self):
        events=trial()[:-3]
        baseline=sample(.15,1,1,0,0,1);baseline['sample']['awake']=1
        events.insert(2,baseline)
        events.append(dict(type='command',version=1,time=2.25,text='PAUSE 1'))
        for time in (2.3,2.51,2.6):
            e=sample(time,1,12,600,-6000,1);e['sample']['awake']=1;events.append(e)
        events.extend([sample(2.7,1,5,600,-6000),sample(3.1,1,5,600,-7000),sample(3.2,1,5,600,-7000)])
        return events

    def test_powered_settle_rejects_missing_baseline_unstable_or_count_change(self):
        events=self.powered_trial();events.pop(2)
        self.assertIn('no fresh powered zero-count baseline',analyzer.analyze(events)['moves'][0]['ratio_rejection_reasons'])
        events=self.powered_trial();events[-4]['sample']['axes'][0]['angle']=-6300
        self.assertIn('unstable settled endpoint',analyzer.analyze(events)['moves'][0]['ratio_rejection_reasons'])
        events=self.powered_trial();events[-4]['sample']['axes'][0]['count']=601
        self.assertIn('pulse count changed during or after powered settlement',analyzer.analyze(events)['moves'][0]['ratio_rejection_reasons'])
        events=self.powered_trial();events[-4]['sample']['awake']=0
        self.assertIn('no fresh powered settled endpoint',analyzer.analyze(events)['moves'][0]['ratio_rejection_reasons'])

    def test_long_preparation_requires_stable_powered_baseline(self):
        events=self.powered_trial();events[1]['text']='JOG 1 0 0 1333 400 1'
        # Shift motion and later events to allow the longer preparation.
        for event in events[3:]:
            event['time']+=.4
            if 'sample' in event:event['sample']['ms']+=400
        for time in (.36,.41,.46):
            e=sample(time,1,1,0,0,1);e['sample']['awake']=1;events.append(e)
        events.sort(key=lambda e:e['time'])
        move=analyzer.analyze(events)['moves'][0]
        self.assertTrue(move['ratio_candidate'],move['ratio_rejection_reasons'])
        self.assertEqual(move['preparation_ms'],400)
        bad=copy.deepcopy(events)
        next(e for e in bad if e['time']==.41)['sample']['axes'][0]['angle']=300
        self.assertIn('unstable powered baseline',analyzer.analyze(bad)['moves'][0]['ratio_rejection_reasons'])
        bad=[e for e in events if e['time']!=.41]
        self.assertIn('insufficient powered baseline settlement',analyzer.analyze(bad)['moves'][0]['ratio_rejection_reasons'])
        events[1]['text']='JOG 1 0 0 1333 501 1'
        self.assertFalse(analyzer.analyze(events)['moves'][0]['accepted'])

    def test_sample_validation(self):
        with tempfile.NamedTemporaryFile(mode='w', suffix='.jsonl') as f:
            f.write('{"version": 1, "time": -1}\n{"version": 1, "time": -2}\n')
            f.flush()
            with self.assertRaises(ValueError):
                analyzer.read_events(f.name)


if __name__ == '__main__':
    unittest.main()
