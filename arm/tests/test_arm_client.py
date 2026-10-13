"""Bounded Mac scripting tests; fake transport/clock, never hardware or SSH."""
import copy
from datetime import datetime, timezone
import uuid
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
import arm_client as client


class Clock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class FakeTransport:
    def __init__(self, clock):
        self.clock = clock
        self.commands = []
        self.closed = False
        self.echo = True
        self.effect = None
        self.fail = None
        self.started = None
        self.s = dict(kind='arm', v=1, error=0, owner=123, state=0, id=0, ms=0,
                      run=0, awake=0, enabled=1, generation=1, configured=7,
                      verified=0, duration=1000, elapsed=0,
                      axes=[dict(angle=0, reference=0, target=0, count=0, direction=0,
                                 age=10, magnet=0x30, sensor_error=0) for _ in range(3)])

    def request(self, command):
        self.commands.append((self.clock(), command))
        words = command.split(); name = words[0]
        if self.fail and name == self.fail[0]: raise self.fail[1]
        if name == 'SHUTDOWN':
            self.s.update(run=0, awake=0)
            return None
        if name in ('JOG', 'POSE'):
            self.started = self.clock()
            self.s.update(run=1, awake=1, state=3, id=int(words[1]))
        if name in ('JOG', 'POSE', 'HOLD') and self.effect:
            self.effect(self, command)
        if name == 'PAUSE':self.s.update(state=12,run=1,awake=1)
        if name == 'STOP':
            self.s.update(run=0, awake=0)
            if self.s['state'] not in (4, 11): self.s['state'] = 5
        result = copy.deepcopy(self.s)
        if name == 'CONFIG' and self.echo:
            result['configuration_echo'] = list(map(int, words[1:]))
        return result

    def close(self): self.closed = True
    def names(self): return [command.split()[0] for _, command in self.commands]
    def angle(self, joint, degrees): self.s['axes'][client.MOTOR[joint]]['angle'] = round(degrees*1000)


class ArmClientTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.clock = Clock()
        self.transport = FakeTransport(self.clock)
        self.session = client.ArmSession(output=Path(self.temp.name)/'record.jsonl',
                                        transport_factory=lambda: self.transport,
                                        clock=self.clock, sleep=self.clock.sleep)

    def test_recording_provenance_utc_uuid_and_injected_relative_clock(self):
        earliest=datetime.now(timezone.utc)
        self.clock.now=123.0
        with self.session as arm:
            self.clock.sleep(1.25)
            arm.status()
        events=[json.loads(line) for line in self.session.output.read_text().splitlines()]
        metadata=events[0]
        self.assertEqual(metadata['type'],'metadata')
        self.assertEqual(metadata['version'],1)
        self.assertEqual(metadata['time'],0)
        self.assertEqual(metadata['fingerprint'],self.session.fingerprint)
        self.assertTrue(metadata['started_at'].endswith('Z'))
        recorded=datetime.fromisoformat(metadata['started_at'].replace('Z','+00:00'))
        self.assertLessEqual(earliest,recorded)
        self.assertLessEqual(recorded,datetime.now(timezone.utc))
        self.assertEqual(uuid.UUID(metadata['session_id']).version,4)
        status_event=next(e for e in events if e['type']=='command' and e['text']=='STATUS')
        self.assertEqual(status_event['time'],1.25)
        other=client.ArmSession(output=Path(self.temp.name)/'another.jsonl',
                                transport_factory=lambda: FakeTransport(self.clock),
                                clock=self.clock,sleep=self.clock.sleep)
        with other:pass
        second=json.loads(other.output.read_text().splitlines()[0])
        self.assertNotEqual(metadata['session_id'],second['session_id'])
        self.assertEqual(second['time'],0)

    def test_finite_bounds_reject_nan_inf_bool_and_non_numeric(self):
        for value in [float('nan'), float('inf'), -float('inf'), True, False, '1', None, -1, 11]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                client.finite(value, 0, 10, 'Value')
        self.assertEqual(client.finite(10, 0, 10, 'Value'), 10)
        self.assertEqual(client.finite(0, 0, 10, 'Value'), 0)

    def ratio_data(self):
        return dict(version=1, fingerprint=self.session.fingerprint, trials={
            name: [dict(direction=d, pulses=600, degrees=6*(1 if j==0 else -1)*(1 if d==0 else -1))
                   for d in (0, 1, 0)] for j, name in enumerate(client.JOINTS)})

    def test_ratio_validation_accepts_consistent_bidirectional_trials(self):
        self.assertEqual(client.validate_ratio_data(self.ratio_data(), self.session.fingerprint), [100, 100, 100])

    def test_ratio_validation_rejects_bad_or_insufficient_measurements(self):
        mutations = [
            lambda d: d.update(fingerprint='different'),
            lambda d: d['trials']['shoulder'].pop(),
            lambda d: [t.update(direction=0, degrees=6) for t in d['trials']['shoulder']],
            lambda d: d['trials']['shoulder'][0].update(direction=False),
            lambda d: d['trials']['shoulder'][0].update(pulses=49),
            lambda d: d['trials']['shoulder'][0].update(pulses=float('nan')),
            lambda d: d['trials']['shoulder'][0].update(pulses=700),
            lambda d: d['trials']['shoulder'][0].update(degrees=4.9),
            lambda d: d['trials']['shoulder'][0].update(degrees=-6),
            lambda d: d['trials']['shoulder'][0].update(degrees=float('inf')),
        ]
        for index, mutate in enumerate(mutations):
            data = self.ratio_data(); mutate(data)
            with self.subTest(index=index), self.assertRaises(ValueError):
                client.validate_ratio_data(data, self.session.fingerprint)

    def test_configuration_echo_required_and_cleanup_on_missing_ack(self):
        self.transport.echo = False
        with self.assertRaisesRegex(client.ArmError, 'acknowledged'):
            self.session.__enter__()
        self.assertEqual(self.transport.names(), ['HELLO', 'CONFIG', 'SHUTDOWN'])
        self.assertTrue(self.transport.closed)

    def test_configuration_mapping_and_no_implicit_enable(self):
        with self.session:
            config = [c for _, c in self.transport.commands if c.startswith('CONFIG')]
            self.assertEqual([int(c.split()[1]) for c in config], [1, 0, 2])
            self.assertEqual([int(c.split()[4]) for c in config], [1, -1, -1])
            self.assertNotIn('POWER', self.transport.names())
        self.assertEqual(self.transport.names()[-1], 'SHUTDOWN')

    def test_invalid_jog_bounds_never_start(self):
        with self.session:
            for kwargs in [dict(seconds=0), dict(seconds=61), dict(seconds=float('nan')),
                           dict(rate_hz=0), dict(rate_hz=19), dict(rate_hz=2401), dict(rate_hz=99.5), dict(rate_hz=True),
                           dict(max_degrees=.01), dict(max_degrees=11), dict(max_degrees=True)]:
                with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                    self.session.jog('shoulder', 'forward', **kwargs)
            for args in [('base', 'forward'), ('elbow', 'left')]:
                with self.assertRaises(ValueError): self.session.jog(*args)
            self.assertNotIn('JOG', self.transport.names())

    def test_powered_settle_freezes_counts_then_sleeps(self):
        def response(t,c):
            if c.startswith('JOG') or t.clock()<.1:t.s.update(state=1,elapsed=250)
            elif t.s['state']!=12:
                t.s.update(state=3);t.angle(1,-1);t.s['axes'][0]['count']=150
        self.transport.effect=response
        with self.session:
            result=self.session.trial('elbow','forward',timeout=.15,powered_settle=True)
        names=self.transport.names()
        self.assertLess(names.index('PAUSE'),names.index('STOP'))
        self.assertEqual(result['powered_initial_angles'],[0,0,0])
        self.assertEqual(result['powered_final_angles'],[0,-1,0])
        self.assertEqual(result['sample']['awake'],0)
        self.assertEqual(result['pulses'],150)

    def test_default_jog_rate_is_100(self):
        with self.session:
            self.session.jog('elbow', 'forward', seconds=.1)
        command=next(c for _,c in self.transport.commands if c.startswith('JOG'))
        self.assertEqual(command.split()[2:5], ['0','0','100'])

    def test_unverified_ratios_block_pose_before_command(self):
        with self.session, self.assertRaisesRegex(client.ArmError, 'ratios'):
            self.session.move_pose([1, 1, 1])
        self.assertNotIn('POSE', self.transport.names())

    def test_pose_requires_firmware_verification_and_small_interior_targets(self):
        self.session.ratios = [10, 10, 10]
        with self.session:
            with self.assertRaisesRegex(client.ArmError, 'ratios'):
                self.session.move_pose([1, 1, 1])
            self.transport.s['verified'] = 7
            for pose in [[6, 0, 0], [38, 0, 0], [0, float('nan'), 0], [0, 0]]:
                with self.subTest(pose=pose), self.assertRaises((ValueError, client.ArmError)):
                    self.session.move_pose(pose)
            self.assertNotIn('POSE', self.transport.names())

    def test_idle_recording_and_context_do_not_send_holds(self):
        with self.session:
            self.session.status()
            self.session.record(.2)
        self.assertNotIn('HOLD', self.transport.names())
        self.assertNotIn('JOG', self.transport.names())
        self.assertGreaterEqual(self.transport.names().count('STATUS'), 2)

    def test_bounded_jog_stops_and_sends_no_holds_during_settling(self):
        with self.session:
            result = self.session.jog('shoulder', 'forward', seconds=.2)
            self.assertEqual(result['reason'], 'deadline')
        names = self.transport.names()
        self.assertIn('HOLD', names)
        self.assertNotIn('HOLD', names[names.index('STOP')+1:])
        stop_time = next(t for t, c in self.transport.commands if c == 'STOP')
        self.assertAlmostEqual(stop_time, .2)
        lines = [json.loads(line) for line in self.session.output.read_text().splitlines()]
        self.assertEqual(lines[0]['type'], 'metadata')
        self.assertTrue(any(line['type'] == 'result' for line in lines))

    def test_jog_stops_at_measured_travel_cap(self):
        def move(t, command): t.angle(0, (self.clock()-t.started)*10)
        self.transport.effect = move
        with self.session:
            result = self.session.jog('shoulder', 'forward', seconds=5, max_degrees=1)
        self.assertEqual(result['reason'], 'travel reached')
        stop_time = next(t for t, c in self.transport.commands if c == 'STOP')
        self.assertLessEqual(stop_time, .15)

    def test_wrong_direction_stops_all(self):
        self.transport.effect = lambda t, command: t.angle(0, -.5)
        with self.session, self.assertRaisesRegex(client.ArmError, 'opposite'):
            self.session.jog('shoulder', 'forward')
        self.assertIn('STOP', self.transport.names())
        self.assertNotIn('HOLD', self.transport.names())

    def test_uncommanded_joint_motion_stops_all(self):
        self.transport.effect = lambda t, command: t.angle(2, 1)
        with self.session, self.assertRaisesRegex(client.ArmError, 'uncommanded'):
            self.session.jog('shoulder', 'forward')
        self.assertIn('STOP', self.transport.names())

    def test_no_response_stalls_after_two_seconds(self):
        with self.session, self.assertRaisesRegex(client.ArmError, '2 seconds'):
            self.session.jog('shoulder', 'forward', seconds=10)
        stop_time = next(t for t, c in self.transport.commands if c == 'STOP')
        self.assertGreaterEqual(stop_time, 2)
        self.assertLess(stop_time, 2.1)

    def test_stall_after_initial_progress_uses_rolling_window(self):
        def move_then_stall(t, command):
            if self.clock() - t.started >= .25: t.angle(0, .25)
        self.transport.effect = move_then_stall
        with self.session, self.assertRaisesRegex(client.ArmError, '2 seconds'):
            self.session.jog('shoulder', 'forward', seconds=10)
        stop_time = next(t for t, c in self.transport.commands if c == 'STOP')
        self.assertGreaterEqual(stop_time, 2.25)
        self.assertLess(stop_time, 2.4)

    def test_settled_endpoint_rejects_wrong_direction_and_other_joint_motion(self):
        for joint, degrees in [(0, -.5), (2, 1)]:
            self.setUp()
            original = self.transport.request
            def drifting_status(command):
                if command == 'STATUS' and 'STOP' in self.transport.names():
                    self.transport.angle(joint, degrees)
                return original(command)
            self.transport.request = drifting_status
            with self.subTest(joint=joint), self.session, self.assertRaisesRegex(client.ArmError, 'settled feedback'):
                self.session.jog('shoulder', 'forward', seconds=.1)
            self.assertIn('STOP', self.transport.names())
            names = self.transport.names()
            self.assertNotIn('HOLD', names[names.index('STOP')+1:])

    def test_stopped_response_still_checks_wrong_direction(self):
        def stop_wrong(t, command):
            t.angle(0, -.5)
            t.s.update(run=0, awake=0, state=11)
        self.transport.effect = stop_wrong
        with self.session, self.assertRaisesRegex(client.ArmError, 'opposite'):
            self.session.jog('shoulder', 'forward')

    def test_stopped_response_still_checks_uncommanded_motion(self):
        def stop_wrong(t, command):
            t.angle(2, 1)
            t.s.update(run=0, awake=0, state=11)
        self.transport.effect = stop_wrong
        with self.session, self.assertRaisesRegex(client.ArmError, 'uncommanded'):
            self.session.jog('shoulder', 'forward')

    def test_sensor_failure_never_renews_lease(self):
        def stale(t, command): t.s['axes'][1]['age'] = 101
        self.transport.effect = stale
        with self.session, self.assertRaisesRegex(client.ArmError, 'Sensor'):
            self.session.jog('shoulder', 'forward')
        self.assertIn('STOP', self.transport.names())
        self.assertNotIn('HOLD', self.transport.names())

    def test_keyboard_interrupt_sends_stop_and_context_release(self):
        self.transport.fail = ('HOLD', KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            with self.session: self.session.jog('shoulder', 'forward')
        self.assertEqual(self.transport.names()[-2:], ['STOP', 'SHUTDOWN'])
        self.assertTrue(self.transport.closed)

    def test_transport_death_attempts_stop_and_closes(self):
        self.transport.fail = ('HOLD', ConnectionError('closed'))
        with self.assertRaises(ConnectionError):
            with self.session: self.session.jog('shoulder', 'forward')
        self.assertEqual(self.transport.names()[-2:], ['STOP', 'SHUTDOWN'])
        self.assertTrue(self.transport.closed)

    def test_stop_failure_closes_transport_for_eof_watchdog(self):
        self.transport.fail = ('STOP', ConnectionError('lost'))
        with self.assertRaises(ConnectionError):
            with self.session: self.session.jog('shoulder', 'forward', seconds=.1)
        self.assertTrue(self.transport.closed)
        self.assertEqual(self.transport.names().count('STOP'), 1)
        self.assertEqual(self.transport.names().count('SHUTDOWN'), 1)

    def test_external_exception_in_context_releases_ownership(self):
        with self.assertRaisesRegex(RuntimeError, 'application error'):
            with self.session: raise RuntimeError('application error')
        self.assertEqual(self.transport.names()[-1], 'SHUTDOWN')
        self.assertTrue(self.transport.closed)


class TransportLinkTests(unittest.TestCase):
    """A real child process stands in for the gateway: silence, exit and error kinds are LinkLost /
    SensorStale quickly, and the transport is closed (EOF makes the MCU release the arm)."""
    def transport(self, code):
        t = client.Transport(command=[sys.executable, '-c', code], timeout=.25)
        t.first = False                       # past start-up: the per-request deadline applies
        self.addCleanup(t.close)
        return t

    def test_silent_gateway_is_link_lost_within_the_deadline(self):
        t = self.transport('import time; time.sleep(30)')
        began = time.monotonic()
        with self.assertRaisesRegex(client.LinkLost, r'No reply .* 0.25 s'):
            t.request('STATUS')
        self.assertLess(time.monotonic() - began, 2.0)     # 0.25 s deadline + bounded close
        self.assertTrue(t.closed)

    def test_gateway_exit_is_link_lost(self):
        t = self.transport('import sys; sys.stderr.write("Arduino router is unavailable"); sys.stdin.readline()')
        with self.assertRaises(client.LinkLost):
            t.request('STATUS')               # replies nothing, exits after reading the request

    def test_dead_gateway_pipe_is_link_lost(self):
        t = self.transport('pass')
        t.process.wait(timeout=5)
        began = time.monotonic()
        with self.assertRaises(client.LinkLost):
            t.request('STATUS')
        self.assertLess(time.monotonic() - began, 2.0)

    def test_error_kinds_map_to_exceptions(self):
        for kind, error in (('link_lost', client.LinkLost), ('sensor_stale', client.SensorStale)):
            reply = json.dumps({'id': 1, 'ok': False, 'error': 'x', 'kind': kind})
            t = self.transport(f'import sys; sys.stdin.readline(); print({reply!r}, flush=True)')
            with self.subTest(kind=kind), self.assertRaises(error):
                t.request('STATUS')
        t = self.transport('import sys; sys.stdin.readline(); print(\'{"id":1,"ok":false,"error":"x"}\', flush=True)')
        with self.assertRaises(client.ArmError) as caught:
            t.request('STATUS')
        self.assertNotIsInstance(caught.exception, (client.LinkLost, client.SensorStale))


if __name__ == '__main__': unittest.main()
