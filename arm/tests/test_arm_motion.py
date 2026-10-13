"""Closed-loop motion API and gateway tests; fake transport/clock, never hardware."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
import arm_client as client
import arm_motion as motion
from arm_gateway import ArmGateway, decode_servo


class Clock:
    def __init__(self): self.now = 0.0
    def __call__(self): return self.now
    def sleep(self, seconds): self.now += seconds


class FakeServo:
    """Firmware stand-in: MOVE runs for `polls` HOLDs, then ends as `finish`."""
    def __init__(self, clock):
        self.clock = clock; self.commands = []; self.closed = False
        self.polls = 4; self.finish = (14, 1); self.reject = {}
        self.s = dict(kind='arm', v=1, error=0, owner=1, state=0, id=0, ms=0, run=0, awake=0, enabled=1,
                      generation=3, configured=7, verified=0, duration=0, elapsed=0,
                      axes=[dict(angle=a, reference=0, target=0, count=0, direction=0, age=10, magnet=0x30,
                                 sensor_error=0) for a in (63281, -12041, 4307)])
        self.servo = dict(kind='servo', v=2, state=0, reason=0, id=0, retries=0, stalls=0, stall_axis=-1,
                          stall_angle=0, duration=0, elapsed=0, scale=1000,
                          axes=[dict(estimate=0, reference=0, goal=0, velocity=0, rate=0, ppd_forward=142200,
                                     ppd_reverse=142200, direction=0, count=0) for _ in range(3)])
        self.left = 0; self.goal = None
        self.roll_supported = True
        self.light_supported = True

    def request(self, command):
        self.commands.append(command)
        words = command.split(); name = words[0]
        if name in self.reject:
            raise client.ArmError(self.reject[name])
        if name == 'SHUTDOWN':
            return None
        if name == 'SERVOSTAT':
            return copy.deepcopy(self.servo)
        if name == 'ROLLSTAT':
            if not self.roll_supported:
                return {'kind': 'roll', 'supported': False}
            return {'kind': 'roll', 'supported': True, 'v': 3, 'enabled': 1, 'target': 12500,
                    'current': 12000, 'pulse_us': 1589, 'fault': 0}
        if name in ('LIGHT', 'LIGHTSTAT'):
            if not self.light_supported:
                return {'kind': 'light', 'supported': False}
            target = int(words[1]) if name == 'LIGHT' else 0
            return {'kind': 'light', 'supported': True, 'v': 4, 'enabled': 1, 'target': target,
                    'current': 0, 'remaining': 0, 'fault': 0, 'refused': 0}
        if name == 'MOVE':
            self.goal = list(map(int, words[2:5]))
            self.left = self.polls
            self.s.update(state=13, run=1, awake=1, id=int(words[1]), duration=2000)
            self.servo.update(state=13, reason=0, id=int(words[1]), duration=2000)
        elif name == 'HOLD' and self.s['state'] == 13:
            self.left -= 1
            if self.left <= 0:
                state, reason = self.finish
                self.s.update(state=state, run=0)
                self.servo.update(state=state, reason=reason)
                if reason == 1:
                    for axis, value in zip(self.s['axes'], self.goal): axis['angle'] = value
        elif name == 'STREAM' and self.s['state'] == 16:
            result = copy.deepcopy(self.s); result['error'] = -16      # stalled: refused until HALT
            return result
        elif name == 'STREAM':
            self.s.update(state=17, run=1, awake=1, id=int(words[1]))
        elif name == 'HALT':
            self.s.update(state=14, run=0); self.servo.update(state=14, reason=9)
        elif name == 'STOP':
            self.s.update(state=0, run=0, awake=0); self.servo.update(state=0, reason=10)
        result = copy.deepcopy(self.s)
        if name == 'CONFIG':
            result['configuration_echo'] = list(map(int, words[1:]))
        return result

    def close(self): self.closed = True
    def sent(self, name): return [c for c in self.commands if c.split()[0] == name]


class ArmMotionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(); self.addCleanup(self.temp.cleanup)
        self.clock = Clock(); self.fake = FakeServo(self.clock)

    def session(self, **kwargs):
        return motion.ArmMotion(output=Path(self.temp.name)/('r%d.jsonl' % len(list(Path(self.temp.name).iterdir()))),
                                transport_factory=lambda: self.fake, clock=self.clock, sleep=self.clock.sleep, **kwargs)

    def test_configures_each_motor_in_motor_order(self):
        with self.session(tuck=30):
            servo = self.fake.sent('SERVO')
            # Motor order elbow, shoulder, head; ppd/speed/accel in thousandths.
            self.assertEqual(servo, ['SERVO 1 146000 12000 40000', 'SERVO 0 140000 12000 40000', 'SERVO 2 143100 12000 40000'])
            self.assertEqual(self.fake.sent('SERVOTIME'), ['SERVOTIME 60000 2 30000'])
        self.assertTrue(self.fake.closed)

    def test_tuck_modes(self):
        with self.session(tuck=False) as arm:
            self.assertEqual(self.fake.sent('SERVOTIME')[-1], 'SERVOTIME 60000 0 0')
            arm.configure(tuck='auto')
            self.assertEqual(self.fake.sent('SERVOTIME')[-1], 'SERVOTIME 60000 1 0')
            with self.assertRaises(ValueError): arm.configure(tuck=78)

    def test_move_maps_joint_order_and_holds_until_complete(self):
        with self.session() as arm:
            result = arm.move_to([-5, 60, 10], speed=8)
        move = self.fake.sent('MOVE')[0].split()
        self.assertEqual(move[2:5], ['60000', '-5000', '10000'])   # elbow, shoulder, head
        self.assertEqual(move[5:], ['0', '8000', '3'])
        self.assertEqual(len(self.fake.sent('HOLD')), 4)
        self.assertTrue(result['ok'])
        self.assertEqual(result['final_angles'], [-5, 60, 10])
        self.assertEqual(result['error_deg'], [0, 0, 0])
        lines = [json.loads(l) for l in Path(result['recording']).read_text().splitlines()]
        self.assertTrue(any(e['type'] == 'servo' for e in lines))
        self.assertTrue(any(e['type'] == 'result' for e in lines))

    def test_duration_is_sent_in_milliseconds(self):
        with self.session() as arm:
            arm.move_to([0, 55, 0], duration=1.5)
        self.assertEqual(self.fake.sent('MOVE')[0].split()[5:7], ['1500', '0'])

    def test_targets_inside_margin_only(self):
        with self.session() as arm:
            for pose in ([37, 0, 0], [0, -71, 0], [0, 0, float('nan')], [0, 0]):
                with self.subTest(pose=pose), self.assertRaises(ValueError):
                    arm.move_to(pose)
            with self.assertRaises(ValueError): arm.move_to([0, 0, 0], speed=0)
            with self.assertRaises(ValueError): arm.move_to([0, 0, 0], duration=61)
        self.assertEqual(self.fake.sent('MOVE'), [])

    def test_stall_is_reported_not_raised(self):
        self.fake.finish = (16, 8)
        self.fake.servo.update(retries=1, stalls=2, stall_axis=1, stall_angle=-1500)
        with self.session() as arm:
            result = arm.move_to([10, 63, 4])
        self.assertFalse(result['ok'])
        self.assertEqual((result['state'], result['reason']), ('stalled', 'obstruction'))
        self.assertEqual((result['stalled_joint'], result['stall_angle']), ('shoulder', -1.5))

    def test_roll_configures_clamps_and_sends_millidegrees(self):
        cfg = dict(center_us=1520, us_per_deg=7.5, limits=(-30, 40), sign=-1, max_speed=120)
        with self.session(roll=cfg) as arm:
            self.assertTrue(arm.roll_supported)
            arm.roll(12.5)
            arm.roll(100, speed=30)                 # clamped to the configured range
            arm.roll(-100)
            status = arm.roll_status()
        self.assertEqual(self.fake.sent('ROLLCONFIG'), ['ROLLCONFIG 1520 7500 -30000 40000 -1 120000'])
        self.assertEqual(self.fake.sent('ROLL'), ['ROLL 12500 0', 'ROLL 40000 30000', 'ROLL -30000 0'])
        self.assertEqual((status['target'], status['current']), (12.5, 12.0))

    def test_roll_is_a_no_op_on_firmware_without_the_servo(self):
        self.fake.roll_supported = False
        with self.session(roll={}) as arm:
            self.assertFalse(arm.roll_supported)
            self.assertIsNone(arm.roll(10))
            arm.roll_off()
            arm.move_to([0, 60, 0])                 # everything else still works
        self.assertEqual(self.fake.sent('ROLLCONFIG') + self.fake.sent('ROLL') + self.fake.sent('ROLLOFF'), [])

    def test_no_roll_config_leaves_the_servo_alone(self):
        with self.session() as arm:
            self.assertFalse(arm.roll_supported)
        self.assertEqual(self.fake.sent('ROLLSTAT'), [])

    def test_light_clamps_formats_and_keeps_the_arm_sample(self):
        with self.session() as arm:
            before = arm.sample
            arm.light(463.6, fade_ms=80)
            arm.light(2000)
            arm.light(-3, fade_ms=1e9)
            self.assertIs(arm.sample, before)       # light replies don't replace the arm status
        self.assertEqual(self.fake.sent('LIGHTSTAT'), ['LIGHTSTAT'])   # asked once
        self.assertEqual(self.fake.sent('LIGHT'), ['LIGHT 464 80', 'LIGHT 1000 0', 'LIGHT 0 60000'])

    def test_light_is_a_no_op_on_firmware_without_the_lamp(self):
        self.fake.light_supported = False
        with self.session() as arm:
            self.assertIsNone(arm.light(500))
            self.assertIsNone(arm.light(600))
            self.assertFalse(arm.light_supported)
        self.assertEqual(self.fake.sent('LIGHT'), [])
        self.assertEqual(len(self.fake.sent('LIGHTSTAT')), 1)

    def test_fault_raises(self):
        self.fake.finish = (18, 4)
        with self.session() as arm, self.assertRaisesRegex(client.ArmError, 'sensor'):
            arm.move_to([0, 60, 0])

    def test_sensor_fault_is_sensor_stale(self):
        self.fake.finish = (18, 4)
        with self.session() as arm, self.assertRaises(client.SensorStale):
            arm.move_to([0, 60, 0])
        self.fake.finish = (18, 5)                 # a limit fault is a plain ArmError
        with self.session() as arm, self.assertRaises(client.ArmError) as caught:
            arm.move_to([0, 60, 0])
        self.assertNotIsInstance(caught.exception, client.SensorStale)

    def test_stale_encoder_while_streaming_is_sensor_stale_not_a_stall(self):
        with self.session() as arm:
            arm.stream_point([1, 62, 5])
            self.fake.s['axes'][client.MOTOR[2]].update(age=250)          # head sample 250 ms old
            with self.assertRaisesRegex(client.SensorStale, r'head \(age 250 ms\)'):
                arm.stream_point([1, 62, 5])
            with self.assertRaises(client.SensorStale):
                arm.angles()                                              # never garbage angles
        self.fake.s['axes'][client.MOTOR[2]].update(age=10)
        with self.session() as arm:
            arm.stream_point([1, 62, 5])
            self.fake.s.update(state=16)
            self.assertEqual(arm.stream_point([1, 62, 4])['state'], 'stalled')   # stall: returned

    def test_stale_joints_explains(self):
        s = copy.deepcopy(self.fake.s)
        s['axes'][client.MOTOR[0]].update(angle=-2**31, sensor_error=3)
        s['axes'][client.MOTOR[1]].update(magnet=0x08)
        self.assertEqual(client.stale_joints(s), ['shoulder (no angle, sensor error 3)', 'elbow (magnet 0x08)'])

    def test_locked_or_unhealthy_arm_refuses(self):
        self.fake.s['enabled'] = 0
        with self.session() as arm, self.assertRaisesRegex(client.ArmError, 'locked'):
            arm.move_to([0, 60, 0])
        self.fake.s['enabled'] = 1; self.fake.s['axes'][0]['sensor_error'] = 3
        with self.session() as arm, self.assertRaisesRegex(client.ArmError, 'feedback'):
            arm.move_to([0, 60, 0])

    def test_deadline_halts(self):
        self.fake.polls = 10**6
        with self.session() as arm, self.assertRaisesRegex(client.ArmError, 'deadline'):
            arm.move_to([0, 60, 0], timeout=1)
        self.assertEqual(len(self.fake.sent('HALT')), 1)

    def test_stream_point_format_and_velocity_bounds(self):
        with self.session() as arm:
            out = arm.stream_point([1, 62, 5], [2, -3, 4], horizon=0.08)
            self.assertEqual(out['state'], 'streaming')
            with self.assertRaises(ValueError): arm.stream_point([1, 62, 5], [13, 0, 0])
        words = self.fake.sent('STREAM')[0].split()
        self.assertEqual(words[2:], ['62000', '1000', '5000', '-3000', '2000', '4000', '80', '3'])

    def test_stream_stall_is_returned_then_halt_resumes(self):
        with self.session() as arm:
            arm.stream_point([1, 62, 5])
            self.fake.s.update(state=16)
            self.assertEqual(arm.stream_point([1, 62, 4])['state'], 'stalled')
            arm.halt()
            self.assertEqual(arm.stream_point([1, 62, 4])['state'], 'streaming')

    def test_scale_halt_relax(self):
        with self.session() as arm:
            arm.scale(0.5)
            self.assertEqual(arm.halt()['reason'], 'halted')
            arm.relax()
            with self.assertRaises(ValueError): arm.scale(0)
        self.assertEqual(self.fake.sent('SCALE'), ['SCALE 500'])
        self.assertTrue(self.fake.sent('STOP'))


class Bridge:
    def __init__(self, servo=None): self.calls = []; self.servo = servo
    def call(self, method, args=None, timeout=None):
        self.calls.append((method, args))
        if method == 'arm_servo_status':
            return self.servo
        return [1, 0] + [0]*36


class GatewayTests(unittest.TestCase):
    def test_new_operation_codes(self):
        bridge = Bridge(); g = ArmGateway(bridge); g.command('HELLO')
        for line, op in [('MOVE 1 1 2 3 0 0 1', 10), ('STREAM 2 1 2 3 -4 5 6 100 1', 11), ('HALT', 12),
                         ('SERVO 0 142200 12000 40000', 13), ('SERVOTIME 60000 1 0', 14), ('SCALE 500', 15)]:
            g.command(line)
            self.assertEqual(bridge.calls[-1][1][0], op)
            self.assertEqual(bridge.calls[-1][1][2:], [int(w) for w in line.split()[1:]])
        with self.assertRaises(ValueError): g.command('MOVE 1 2 3')

    def test_servo_status_decoding(self):
        raw = [2, 14, 1, 7, 0, 0, -1, 0, 1500, 1600, 1000] + list(range(27))
        g = ArmGateway(Bridge(raw)); g.command('HELLO')
        s = g.command('SERVOSTAT')
        self.assertEqual((s['state'], s['reason'], s['duration']), (14, 1, 1500))
        self.assertEqual(s['axes'][1]['estimate'], 9)
        self.assertEqual(s['axes'][2]['count'], 26)
        for bad in ([1]*38, raw[:-1], raw[:-1]+[1.5]):
            with self.assertRaises(ValueError): decode_servo(bad)


if __name__ == '__main__':
    unittest.main()
