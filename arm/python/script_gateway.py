#!/usr/bin/env python3
"""Request/reply transport for scripts; never polls or generates motion HOLDs.

Every newline-terminated JSON request produces exactly one JSON response. A bad
request or failed operation terminates the session and stops/releases its owner.
The MCU's independent deadman remains the fallback if communication is lost.
"""
import argparse
import errno
import json
import sys

from arm_gateway import AXIS, HEADER, ArmGateway
from gateway import Gateway

MAX_LINE = 2048  # Bytes excluding the final newline.
# Firmware errors that leave the session usable and are returned with their status
# instead of ending it. A STREAM refused with -EBUSY means the servo stalled and holds
# where it is; the client acknowledges with HALT. Releasing here would sleep the drivers
# and let the arm sag.
NONFATAL = {('STREAM', -errno.EBUSY)}


def snapshot(bridge, method):
    """Decode an explicit group RPC without claiming ownership."""
    raw = bridge.call(method, timeout=0.2)
    if (not isinstance(raw, (list, tuple)) or len(raw) != 38
            or raw[0] != 1 or any(type(value) is not int for value in raw)):
        raise ValueError('Coordinated observation firmware required')
    result = dict(zip(HEADER, raw[:14]))
    result['axes'] = [dict(zip(AXIS, raw[14+8*a:22+8*a])) for a in range(3)]
    result['kind'] = 'arm'
    return result


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON key')
        result[key] = value
    return result


def serve(gateway, source=None, destination=None):
    """Serve binary input/text output; return 0 on EOF/shutdown, 1 on error."""
    source = sys.stdin.buffer if source is None else source
    destination = sys.stdout if destination is None else destination
    last_id = 0
    cleaned = False

    def cleanup():
        nonlocal cleaned
        if not cleaned:
            cleaned = True
            gateway.stop()

    def reply(value):
        destination.write(json.dumps(value, separators=(',', ':'), allow_nan=False) + '\n')
        destination.flush()

    try:
        while True:
            request_id = None
            try:
                raw = source.readline(MAX_LINE + 2)
                if not raw:
                    return 0
                if not raw.endswith(b'\n'):
                    raise ValueError('Request exceeds 2048 bytes or lacks a final newline')
                if len(raw) - 1 > MAX_LINE:
                    raise ValueError('Request exceeds 2048 bytes')
                request = json.loads(raw, object_pairs_hook=unique_object)
                if isinstance(request, dict) and type(request.get('id')) is int and request['id'] > 0:
                    request_id = request['id']
                if not isinstance(request, dict) or set(request) != {'id', 'command'}:
                    raise ValueError('Request must contain only id and command')
                if request_id is None:
                    raise ValueError('id must be a positive integer')
                if request_id <= last_id:
                    raise ValueError('id must increase for every request')
                last_id = request_id
                command = request['command']
                if (not isinstance(command, str) or not command.strip()
                        or any(ord(char) < 32 or ord(char) > 126 for char in command)):
                    raise ValueError('command must be a nonempty ASCII command on one line')
                command = command.strip()
                if command == 'SHUTDOWN':
                    cleanup()
                    reply({'id': request_id, 'ok': True, 'status': None})
                    return 0
                if command in ('OBSERVE', 'LOCK'):
                    if not isinstance(gateway, ArmGateway):
                        raise ValueError(command + ' requires group mode')
                    status = snapshot(gateway.bridge, 'arm_observe' if command == 'OBSERVE' else 'arm_lock')
                else:
                    status = gateway.command(command)
                if not isinstance(status, dict):
                    raise ValueError('Command returned no status')
                if status.get('error', 0) and (command.split()[0], status['error']) not in NONFATAL:
                    raise RuntimeError('Firmware rejected command: %s' % status['error'])
                reply({'id': request_id, 'ok': True, 'status': status})
            except Exception as exc:
                # Cleanup precedes the error reply, so a receiving client can
                # reconnect knowing release was already attempted.
                try:
                    cleanup()
                except Exception:
                    pass
                reply({'id': request_id, 'ok': False, 'error': str(exc)})
                return 1
    finally:
        try:
            cleanup()
        except Exception:
            pass  # MCU motion lease expires independently after 300 ms.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--axis', type=int, choices=range(3))
    mode.add_argument('--group', action='store_true', help='All joints (default)')
    args = parser.parse_args()
    from arduino.router_bridge import Bridge
    bridge = Bridge()
    try:
        if not bridge.connect(timeout=3):
            raise ConnectionError('Arduino router is unavailable')
        gateway = ArmGateway(bridge) if args.axis is None else Gateway(bridge, args.axis)
        return serve(gateway)
    except Exception as exc:
        print('Scripting transport: ' + str(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        bridge.disconnect()


if __name__ == '__main__':
    sys.exit(main())
