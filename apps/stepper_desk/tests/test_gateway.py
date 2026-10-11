import contextlib
import io
import os
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from gateway import Gateway, serve

class FakeBridge:
    def __init__(self): self.calls = []
    def call(self, method, values, timeout):
        self.calls.append(values)
        return [3, values[1], values[3] if values[0] == 1 else 0, 0, 0, 0, 0, 0, 0, 0, 200, 0, 2400, 0]

class Tests(unittest.TestCase):
    def setUp(self):
        self.bridge = FakeBridge()
        self.g = Gateway(self.bridge, 2)
    def test_sensor_telemetry(self):
        class SensorBridge(FakeBridge):
            def call(self, *args, **kwargs):
                state = super().call(*args, **kwargs)
                state[0] = 4
                return state + [2048, 32, 0, 15]
        result = Gateway(SensorBridge(), 1).command('HELLO')
        self.assertEqual(result['angle_raw'], 2048)
        self.assertEqual(result['magnet_status'], 32)
        self.assertEqual(result['sensor_error'], 0)
        self.assertEqual(result['sensor_age_ms'], 15)
    def test_limit_configuration_and_telemetry(self):
        class LimitBridge(FakeBridge):
            def call(self, *args, **kwargs):
                state=super().call(*args, **kwargs); state[0]=5
                return state+[2048,48,0,15,0,1,-30322266,1,-1,10000,-80000,87000,1]
        bridge=LimitBridge(); gateway=Gateway(bridge,2)
        result=gateway.command('HELLO')
        self.assertEqual(result['limit_state'],0)
        self.assertEqual(result['limit_revision'],1)
        result=gateway.command('LIMITS -30322266 1 -1')
        self.assertEqual(bridge.calls[-1],[5,2,gateway.owner,-30322266,1,-1])
        for bad in ['LIMITS 0 0 0','LIMITS 180000001 0 1','LIMITS 0 2 -1','LIMITS --1 0 1','LIMITS -1 0 1 9']:
            with self.assertRaises(ValueError):gateway.command(bad)
    def test_requires_claim(self):
        with self.assertRaises(ValueError): self.g.command('CURVE 1 0 200 2 0 0 60 500')
        self.assertEqual(self.bridge.calls, [])
    def test_routes_axis_owner_and_curve(self):
        self.g.command('HELLO')
        self.g.command('CURVE 1 1 400 3 0 0 60 1000 3000 2000')
        self.assertEqual(self.bridge.calls[-1], [1,2,self.g.owner,1,1,400,3,0,0,60,1000,3000,2000])
        self.g.command('HOLD 1')
        self.assertEqual(self.bridge.calls[-1], [2,2,self.g.owner,1])
    def test_rejects_malformed_without_rpc(self):
        self.g.command('HELLO')
        for command in ['CURVE 1 0 200 9 0 0', 'HOLD -1', 'HOLD 99999999999', 'STOP 1', 'hello', 'HOLD 1 2']:
            with self.assertRaises(ValueError): self.g.command(command)
        self.assertEqual(len(self.bridge.calls), 1)
    def test_stop_supersedes_queued_start_and_eof_stops(self):
        self.g.command('HELLO')
        read, write = os.pipe()
        os.write(write, b'CURVE 1 0 200 2 0 0 60 500\nHOLD 1\nSTOP\n')
        os.close(write)
        with contextlib.redirect_stdout(io.StringIO()): serve(self.g, read)
        os.close(read)
        self.assertEqual([a[0] for a in self.bridge.calls], [0,3,3])
    def test_hello_and_stop_same_batch(self):
        read, write = os.pipe()
        os.write(write, b'HELLO\nCURVE 1 0 200 2 0 0 60 500\nSTOP\n')
        os.close(write)
        with contextlib.redirect_stdout(io.StringIO()): serve(self.g, read)
        os.close(read)
        self.assertEqual([a[0] for a in self.bridge.calls], [0,3,3])
    def test_poll_does_not_renew(self):
        self.g.command('HELLO'); self.g.command('STATUS'); self.g.command('STATUS')
        self.assertEqual([a[0] for a in self.bridge.calls], [0,4,4])

if __name__ == '__main__': unittest.main()
