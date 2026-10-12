"""Request/reply tests with no hardware access or real motion."""
import errno
import io
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from arm_gateway import ArmGateway
from gateway import Gateway
from script_gateway import serve


class FakeBridge:
    def __init__(self):
        self.calls = []
        self.fail_op = None
        self.reject_op = None
        self.observe_response = None
        self.roll_response = [3, 1, 10000, 9500, 1570, 0]

    def call(self, method, *args, timeout):
        self.calls.append((method, args, timeout))
        if method in ('arm_observe', 'arm_lock'):
            if self.observe_response is not None:
                return self.observe_response
            return [1, 0, 456, 0, 7, 1000, 0, 0, 1, 9, 7, 0, 5000, 0] + [123, 124, 125, 0, 1, 10, 0x30, 0]*3
        if method == 'arm_roll_status':
            if self.roll_response is None:
                raise ValueError('Unknown method arm_roll_status')   # firmware without the servo
            return self.roll_response
        values = args[0]
        op = values[0]
        if op == self.fail_op:
            raise TimeoutError('Fake RPC timeout')
        error = -errno.EPERM if op == self.reject_op else 0
        if method == 'stepper_command':
            return [7, values[1], 0, error, 0, 0, 0, 0, 0, 0, 400, 0, 2400, values[2]] + [0]*17
        return [1, error, values[1], 0, 7, 1000, 0, 0, 1, 9, 7, 0, 5000, 0] + [123, 124, 125, 0, 1, 10, 0x30, 0]*3


def request(number, command):
    return json.dumps({'id': number, 'command': command}).encode() + b'\n'


class ScriptGatewayTests(unittest.TestCase):
    def setUp(self):
        self.bridge = FakeBridge()
        self.gateway = ArmGateway(self.bridge)

    def run_server(self, data, gateway=None):
        output = io.StringIO()
        result = serve(gateway or self.gateway, io.BytesIO(data), output)
        return result, [json.loads(line) for line in output.getvalue().splitlines()]

    def ops(self):
        return [args[0][0] for name, args, timeout in self.bridge.calls if name == 'arm_command']

    def test_request_correlation_and_eof_cleanup(self):
        result, replies = self.run_server(request(1, 'HELLO') + request(3, 'JOG 1 0 0 9') + request(4, 'STATUS'))
        self.assertEqual(result, 0)
        self.assertEqual([r['id'] for r in replies], [1, 3, 4])
        self.assertTrue(all(r['ok'] for r in replies))
        self.assertEqual(self.ops(), [0, 3, 4, 6, 7])
        self.assertFalse(self.gateway.claimed)

    def test_shutdown_stops_and_releases_once_ignores_following_input(self):
        result, replies = self.run_server(request(1, 'HELLO') + request(2, 'SHUTDOWN') + request(3, 'JOG 1 0 0 9'))
        self.assertEqual(result, 0)
        self.assertEqual(replies[-1], {'id': 2, 'ok': True, 'status': None})
        self.assertEqual(self.ops(), [0, 6, 7])
        self.assertEqual(len(replies), 2)

    def test_read_only_observe_without_hello_or_cleanup_rpc(self):
        result, replies = self.run_server(request(1, 'OBSERVE') + request(2, 'OBSERVE'))
        self.assertEqual(result, 0)
        self.assertEqual(len(self.bridge.calls), 2)
        self.assertTrue(all(call == ('arm_observe', (), .2) for call in self.bridge.calls))
        self.assertFalse(self.gateway.claimed)
        self.assertEqual(replies[0]['status']['owner'], 456)
        self.assertEqual(replies[0]['status']['axes'][2]['angle'], 123)

    def test_explicit_lock_without_hello_uses_only_shared_lock_rpc(self):
        code, replies = self.run_server(request(1, 'LOCK'))
        self.assertEqual(code, 0)
        self.assertTrue(replies[0]['ok'])
        self.assertEqual(self.bridge.calls, [('arm_lock', (), .2)])
        self.assertFalse(self.gateway.claimed)

    def test_lock_not_available_on_axis(self):
        code, replies = self.run_server(request(1, 'LOCK'), Gateway(self.bridge, 0))
        self.assertEqual(code, 1)
        self.assertFalse(replies[0]['ok'])
        self.assertEqual(self.bridge.calls, [])

    def test_observe_rejects_corrupt_firmware_snapshot(self):
        for response in [[], [1]*37, [2]*38, [1]*37+[True], [1]*37+[1.0]]:
            self.bridge.observe_response = response
            with self.subTest(response=response):
                code, replies = self.run_server(request(1, 'OBSERVE'))
                self.assertEqual(code, 1)
                self.assertFalse(replies[0]['ok'])
        self.assertEqual(self.ops(), [])

    def test_observe_not_available_on_axis(self):
        code, replies = self.run_server(request(1, 'OBSERVE'), Gateway(self.bridge, 0))
        self.assertEqual(code, 1)
        self.assertFalse(replies[0]['ok'])
        self.assertEqual(self.bridge.calls, [])

    def test_no_implicit_poll_hold_power_or_claim(self):
        self.assertEqual(self.run_server(b''), (0, []))
        self.assertEqual(self.bridge.calls, [])
        self.run_server(request(1, 'HELLO') + request(2, 'STATUS') + request(3, 'HOLD 9'))
        self.assertEqual(self.ops(), [0, 4, 5, 6, 7])

    def test_legacy_axis_commands_use_existing_gateway(self):
        code, replies = self.run_server(request(1, 'HELLO') + request(2, 'STATUS'), Gateway(self.bridge, 2))
        self.assertEqual(code, 0)
        self.assertEqual(len(replies), 2)
        self.assertEqual([c[1][0][0] for c in self.bridge.calls], [0, 4, 3])
        self.assertTrue(all(c[1][0][1] == 2 for c in self.bridge.calls))

    def test_invalid_request_terminates_and_releases(self):
        invalid = [
            b'{bad}\n', b'{}\n', b'[]\n', b'null\n',
            b'{"id":true,"command":"STATUS"}\n',
            b'{"id":0,"command":"STATUS"}\n',
            b'{"id":-1,"command":"STATUS"}\n',
            b'{"id":1.5,"command":"STATUS"}\n',
            b'{"id":2,"command":null}\n',
            b'{"id":2,"command":"STATUS","extra":1}\n',
            b'{"id":2,"id":3,"command":"STATUS"}\n',
            request(2, ''), request(2, 'STATUS\nHOLD 1'), request(2, 'STATUS\x00'),
            request(2, 'UNKNOWN'), request(2, 'HOLD --1'), request(2, 'SHUTDOWN 1'),
            request(2, 'X'*2048), request(2, 'STATUS')[:-1], b'\xff\n',
        ]
        for data in invalid:
            self.setUp()
            with self.subTest(data=data[:80]):
                code, replies = self.run_server(request(1, 'HELLO') + data + request(4, 'JOG 1 0 0 9'))
                self.assertEqual(code, 1)
                self.assertEqual(len(replies), 2)
                self.assertFalse(replies[-1]['ok'])
                self.assertEqual(self.ops(), [0, 6, 7])

    def test_duplicate_or_decreasing_id_cannot_replay_motion(self):
        for request_id in [1, 2]:
            self.setUp()
            code, replies = self.run_server(request(2, 'HELLO') + request(request_id, 'JOG 1 0 0 9'))
            self.assertEqual(code, 1)
            self.assertEqual(replies[-1]['id'], request_id)
            self.assertEqual(self.ops(), [0, 6, 7])

    def test_incomplete_line_at_eof_never_executes(self):
        code, replies = self.run_server(request(1, 'HELLO') + request(2, 'JOG 1 0 0 9')[:-1])
        self.assertEqual(code, 1)
        self.assertEqual(self.ops(), [0, 6, 7])

    def test_firmware_rejection_is_failure_and_stops(self):
        self.bridge.reject_op = 3
        code, replies = self.run_server(request(1, 'HELLO') + request(2, 'JOG 1 0 0 9'))
        self.assertEqual(code, 1)
        self.assertIn(str(-errno.EPERM), replies[-1]['error'])
        self.assertEqual(self.ops(), [0, 3, 6, 7])

    def test_rpc_failure_still_releases(self):
        self.bridge.fail_op = 4
        code, replies = self.run_server(request(1, 'HELLO') + request(2, 'STATUS'))
        self.assertEqual(code, 1)
        self.assertEqual(self.ops(), [0, 4, 6, 7])

    def test_cleanup_failure_attempts_release_without_second_stop(self):
        self.bridge.fail_op = 6
        code, replies = self.run_server(request(1, 'HELLO') + request(2, 'SHUTDOWN'))
        self.assertEqual(code, 1)
        self.assertFalse(replies[-1]['ok'])
        self.assertEqual(self.ops(), [0, 6, 7])

    def test_roll_commands_map_to_ops_16_to_18(self):
        bridge = FakeBridge(); gateway = ArmGateway(bridge)
        gateway.command('HELLO')
        gateway.command('ROLL 12000 30000')
        gateway.command('ROLLCONFIG 1500 7407 -45000 45000 1 180000')
        gateway.command('ROLLOFF')
        ops = [c[1][0][:2] for c in bridge.calls if c[0] == 'arm_command'][1:]
        self.assertEqual([o[0] for o in ops], [16, 17, 18])
        self.assertEqual(bridge.calls[2][1][0][2:], [1500, 7407, -45000, 45000, 1, 180000])
        with self.assertRaises(ValueError):
            gateway.command('ROLL 12000')            # wrong argument count

    def test_roll_status_decodes_or_reports_unsupported(self):
        bridge = FakeBridge(); gateway = ArmGateway(bridge)
        gateway.command('HELLO')
        status = gateway.command('ROLLSTAT')
        self.assertEqual((status['supported'], status['enabled'], status['target'], status['pulse_us']),
                         (True, 1, 10000, 1570))
        bridge.roll_response = None
        self.assertEqual(gateway.command('ROLLSTAT'), {'kind': 'roll', 'supported': False})
        bridge.roll_response = [2, 0, 0]               # wrong shape: treated as no servo
        self.assertFalse(gateway.command('ROLLSTAT')['supported'])

    def test_response_is_flushed_for_each_request(self):
        class Output(io.StringIO):
            def __init__(self):
                super().__init__()
                self.flushes = 0
            def flush(self):
                self.flushes += 1
        output = Output()
        serve(self.gateway, io.BytesIO(request(1, 'OBSERVE') + request(2, 'SHUTDOWN')), output)
        self.assertEqual(output.flushes, 2)


if __name__ == '__main__':
    unittest.main()
