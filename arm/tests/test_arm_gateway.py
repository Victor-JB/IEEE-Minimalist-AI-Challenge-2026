"""Exercise the all-joint transport with fake RPC and real pipe queue handling.

These tests never connect to the board and never energize hardware.
"""
import contextlib
import errno
import io
import json
import os
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from arm_gateway import ArmGateway
from gateway import emit, serve


class FakeBridge:
    def __init__(self):
        self.calls = []
        self.error = 0
        self.response = None
        self.fail_op = None

    def call(self, method, args, timeout):
        self.calls.append((method, list(args), timeout))
        if args[0] == self.fail_op:
            raise TimeoutError('test RPC timeout')
        if self.response is not None:
            return self.response
        header = [1, self.error, args[1], 0, 23, 12345, 0, 0, 1, 7, 7, 0, 5000, 42]
        axes = []
        for axis in range(3):
            axes += [1000*axis, 1000*axis+1, 1000*axis+2, 200+axis, axis%2, 10+axis, 0x30, 0]
        return header + axes


class ArmGatewayTests(unittest.TestCase):
    def setUp(self):
        self.bridge = FakeBridge()
        self.gateway = ArmGateway(self.bridge)

    def ops(self):
        return [call[1][0] for call in self.bridge.calls]

    def claim(self):
        return self.gateway.command('HELLO')

    def pipe(self, commands):
        read, write = os.pipe()
        try:
            os.write(write, commands)
            os.close(write)
            write = None
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                serve(self.gateway, read)
            return output.getvalue()
        finally:
            os.close(read)
            if write is not None:
                os.close(write)

    def test_decode_preserves_motor_order_and_protocol(self):
        result = self.claim()
        self.assertTrue(self.gateway.claimed)
        self.assertEqual(result['kind'], 'arm')
        self.assertEqual(result['verified'], 0)
        self.assertEqual(result['generation'], 7)
        self.assertEqual([a['angle'] for a in result['axes']], [0, 1000, 2000])
        self.assertEqual([a['count'] for a in result['axes']], [200, 201, 202])
        self.assertEqual([a['age'] for a in result['axes']], [10, 11, 12])
        method, args, timeout = self.bridge.calls[0]
        self.assertEqual(method, 'arm_command')
        self.assertEqual(args, [0, self.gateway.owner])
        self.assertTrue(0 < self.gateway.owner <= 2**31-1)
        self.assertEqual(timeout, .2)

    def test_all_commands_route_with_same_owner(self):
        self.claim()
        commands = [
            ('CONFIG 1 22500000 0 1 177778 1', [1, 1, 22500000, 0, 1, 177778, 1]),
            ('POSE 24 -1000 2000 -3000 7', [2, 24, -1000, 2000, -3000, 7]),
            ('JOG 25 2 1 7', [3, 25, 2, 1, 7]),
            ('JOG 26 0 0 100 7', [3, 26, 0, 0, 100, 7]),
            ('PAUSE 26', [9,26]),
            ('JOG 27 0 0 1333 400 7', [3,27,0,0,1333,400,7]),
            ('STATUS', [4]), ('HOLD 25', [5, 25]), ('STOP', [6]), ('POWER 0 7', [8, 0, 7]),
        ]
        for command, expected in commands:
            with self.subTest(command=command):
                self.gateway.command(command)
                self.assertEqual(self.bridge.calls[-1][1], [expected[0], self.gateway.owner, *expected[1:]])

    def test_no_motion_or_cleanup_without_hello(self):
        for command in ['POSE 1 0 0 0 1', 'JOG 1 0 0 1', 'HOLD 1', 'STATUS', 'STOP']:
            with self.assertRaises(ValueError):
                self.gateway.command(command)
        self.gateway.stop()
        self.assertEqual(self.bridge.calls, [])
        self.assertIsNone(self.gateway.command(' \t'))

    def test_busy_claim_is_not_owned(self):
        self.bridge.error = -errno.EBUSY
        with self.assertRaises(RuntimeError):
            self.claim()
        self.assertFalse(self.gateway.claimed)
        self.gateway.stop()
        self.assertEqual(self.ops(), [0])

    def test_firmware_errors_and_ownership_loss(self):
        self.claim()
        self.bridge.error = -errno.EPERM
        self.assertEqual(self.gateway.command('POSE 1 0 0 0 1')['error'], -errno.EPERM)
        self.bridge.error = -errno.EACCES
        with self.assertRaises(ConnectionError):
            self.gateway.command('STATUS')

    def test_reject_incompatible_or_corrupt_status(self):
        valid = self.bridge.call('arm_command', [0, 1], .2)
        bad_version = valid.copy(); bad_version[0] = 2
        boolean_field = valid.copy(); boolean_field[3] = False
        floating_field = valid.copy(); floating_field[14] = 1.0
        for response in [None, {}, 'data', valid[:-1], valid+[0], bad_version, boolean_field, floating_field]:
            # None is a valid "use default" sentinel in FakeBridge, so return it explicitly.
            class Reply:
                def call(self, *args, **kwargs):
                    return response
            with self.subTest(response=repr(response)[:60]):
                with self.assertRaises(ValueError):
                    ArmGateway(Reply()).command('HELLO')

    def test_malformed_arguments_never_reach_rpc(self):
        self.claim()
        for command in [
            'HELLO', 'hello', 'HOLD', 'HOLD 1 2', 'STOP 1', 'STATUS 2',
            'POSE 1 0 0 0', 'JOG 1 0 0 1 7', 'CONFIG 0 0 0 1 0',
            'HOLD --1', 'HOLD 1-2', 'HOLD +1', 'HOLD 1.0', 'HOLD ١',
            'HOLD 2147483648', 'HOLD -2147483648', 'HOLD 999999999999',
            'POWER 1 1 garbage', 'REPLAY recording.jsonl', 'eval 1',
        ]:
            with self.subTest(command=command):
                with self.assertRaises(ValueError):
                    self.gateway.command(command)
        self.assertEqual(self.ops(), [0])

    def test_poll_never_renews_deadman(self):
        self.claim()
        for _ in range(10):
            self.gateway.command('STATUS')
        self.assertEqual(self.ops(), [0]+[4]*10)
        self.assertNotIn(5, self.ops())

    def test_eof_stops_then_releases(self):
        self.pipe(b'HELLO\nJOG 1 0 0 1\n')
        self.assertEqual(self.ops(), [0, 3, 6, 7])

    def test_queued_stop_discards_unexecuted_motion_and_hold(self):
        self.pipe(b'HELLO\nPOSE 1 0 0 0 1\nHOLD 1\nSTOP\n')
        self.assertEqual(self.ops(), [0, 6, 6, 7])

    def test_queued_power_lock_discards_jog_and_hold(self):
        self.pipe(b'HELLO\nJOG 1 0 0 1\nHOLD 1\nPOWER 0 1\n')
        self.assertEqual(self.ops(), [0, 8, 6, 7])

    def test_partial_command_at_eof_is_not_executed(self):
        self.pipe(b'HELLO\nJOG 1 0 0 1')
        self.assertEqual(self.ops(), [0, 6, 7])

    def test_corrupt_input_still_stops_and_releases(self):
        with self.assertRaises(ValueError):
            self.pipe(b'HELLO\nJOG malformed\n')
        self.assertEqual(self.ops(), [0, 6, 7])

    def test_release_is_attempted_after_stop_timeout(self):
        self.claim()
        self.bridge.fail_op = 6
        with self.assertRaises(TimeoutError):
            self.gateway.stop()
        self.assertEqual(self.ops(), [0, 6, 7])

    def test_json_telemetry_is_data_only_and_versioned(self):
        data = self.claim()
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            emit(data)
        text = output.getvalue()
        self.assertTrue(text.startswith('@ARM '))
        self.assertEqual(json.loads(text[5:]), data)
        self.assertEqual(data['v'], 1)
        self.assertEqual(self.ops(), [0])


if __name__ == '__main__':
    unittest.main()
