#!/usr/bin/env python3
"""One SSH session owns one motor; only actual Mac HOLD messages renew motion."""
import argparse
import errno
import json
import os
import secrets
import selectors
import sys
import time

PHASES = ['READY', 'ARMING', 'RAMPING', 'RUNNING', 'STOPPED', 'LINK_LOST', 'ERROR', 'LIMIT_STOP']

class Gateway:
    def __init__(self, bridge, axis):
        self.bridge, self.axis = bridge, axis
        self.owner = secrets.randbelow(2**31-1)+1
        self.claimed = False

    def rpc(self, op, *args):
        raw = self.bridge.call('stepper_command', [op, self.axis, self.owner, *args], timeout=0.2)
        if not isinstance(raw, (list, tuple)) or not raw or (raw[0], len(raw)) not in ((3, 14), (4, 18), (5, 27), (6, 30), (7, 31)) or raw[1] != self.axis:
            raise ValueError('Incompatible Arduino Q firmware')
        if raw[3] == -errno.EACCES:
            raise ConnectionError('Motor ownership changed; reconnect to claim it')
        return dict(zip(['v', 'axis', 'ack', 'error', 'fault', 'hz', 'target', 'dir',
                         'want_dir', 'run', 'ppr', 'phase', 'max_hz', 'session',
                         'angle_raw', 'magnet_status', 'sensor_error', 'sensor_age_ms',
                         'limit_state', 'limit_configured', 'limit_zero_udeg', 'limit_reversed',
                         'limit_forward_sign', 'joint_angle_mdeg', 'limit_min_mdeg', 'limit_max_mdeg', 'limit_revision', 'drivers_awake', 'power_generation', 'power_ready', 'power_enabled'], raw))

    def command(self, line):
        parts = line.split()
        if not parts:
            return None
        command, words = parts[0], parts[1:]
        if command == 'HELLO' and not words and not self.claimed:
            result = self.rpc(0)
            if result['error']:
                raise RuntimeError('Motor is busy or has a hardware fault: %s' % result['error'])
            self.claimed = True
            return result
        if not self.claimed:
            raise ValueError('HELLO required before motor commands')
        if command in ('STOP', 'STATUS') and not words:
            return self.rpc(3 if command == 'STOP' else 4)
        if command == 'POWER':
            if len(words) != 2 or words[0] not in ('0','1') or not words[1].isascii() or not words[1].isdecimal() or len(words[1])>10 or not 1 <= int(words[1]) <= 2**31-1:
                raise ValueError('Malformed shared power command')
            return self.rpc(6, *map(int, words))
        if command == 'LIMITS':
            if len(words) != 3 or any(not w.isascii() or not w.lstrip('-').isdecimal() or len(w)>11 or w.count('-')>1 or ('-' in w and not w.startswith('-')) for w in words):
                raise ValueError('Malformed limit calibration')
            zero, reverse, forward = map(int, words)
            if not -180000000 <= zero <= 180000000 or reverse not in (0,1) or forward not in (-1,1):
                raise ValueError('Invalid limit calibration')
            return self.rpc(5, zero, reverse, forward)
        if command not in ('CURVE', 'HOLD'):
            raise ValueError('Unknown command')
        if any(not w.isascii() or not w.isdecimal() or len(w) > 10 for w in words):
            raise ValueError('Command arguments must be nonnegative integers')
        args = list(map(int, words))
        if any(v > 2**31-1 for v in args):
            raise ValueError('Argument too large')
        if command == 'HOLD' and len(args) == 1:
            return self.rpc(2, *args)
        if command == 'CURVE' and len(args) >= 4 and 2 <= args[3] <= 8 and len(args) in (4+2*args[3], 5+2*args[3]):
            return self.rpc(1, *args)
        raise ValueError('Malformed command')

    def stop(self):
        if self.claimed:
            self.rpc(3)


def emit(status):
    if status is None:
        return
    status = dict(status)
    if status.get('kind') == 'arm':
        print('@ARM ' + json.dumps(status, separators=(',', ':')), flush=True)
        return
    status['v'] = 2  # Mac telemetry schema; RPC firmware schema is independently v3/v4.
    status['phase'] = PHASES[status['phase']] if 0 <= status['phase'] < len(PHASES) else 'ERROR'
    print('@STP ' + json.dumps(status, separators=(',', ':')), flush=True)


def serve(gateway, fd=0):
    selector = selectors.DefaultSelector()
    selector.register(fd, selectors.EVENT_READ)
    buffer = b''
    next_status = time.monotonic() + 0.1
    try:
        while True:
            if selector.select(max(0, next_status-time.monotonic())):
                chunk = os.read(fd, 4096)
                if not chunk:
                    return
                buffer += chunk
                if len(buffer) > 4096:
                    raise ValueError('Command queue exceeded limit')
                lines = buffer.split(b'\n')
                buffer = lines.pop()
                # A queued STOP supersedes every earlier command, including a
                # delayed start. Commands after it still need a fresh CURVE id.
                stops = [i for i, line in enumerate(lines) if line.strip() == b'STOP' or line.strip().startswith(b'POWER ')]
                if stops:
                    if not gateway.claimed and any(line.strip() == b'HELLO' for line in lines[:stops[-1]]):
                        emit(gateway.command('HELLO'))
                    lines = lines[stops[-1]:]
                for raw in lines:
                    if len(raw) > 512:
                        raise ValueError('Command too long')
                    emit(gateway.command(raw.decode('ascii')))
                if len(buffer) > 512:
                    raise ValueError('Command too long')
            if time.monotonic() >= next_status:
                if gateway.claimed:
                    emit(gateway.rpc(4))
                next_status = time.monotonic()+0.1
    finally:
        selector.close()
        try:
            gateway.stop()
        except Exception:
            pass  # MCU's independent 300 ms lease still expires.


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--axis', type=int, choices=range(3))
    group.add_argument('--group', action='store_true')
    args = parser.parse_args()
    from arduino.router_bridge import Bridge
    bridge = Bridge()
    try:
        if not bridge.connect(timeout=3):
            raise ConnectionError('Arduino router is unavailable')
        if args.group:
            from arm_gateway import ArmGateway
            serve(ArmGateway(bridge))
        else:
            serve(Gateway(bridge, args.axis))
    except Exception as exc:
        print('@ERR ' + str(exc), flush=True)
        return 1
    finally:
        bridge.disconnect()
    return 0

if __name__ == '__main__':
    sys.exit(main())
