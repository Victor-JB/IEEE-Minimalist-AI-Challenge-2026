"""Closed-loop motion API for behavior code. Mac side; no background HOLDs.

The STM32 closes the loop on the AS5600 angles (sketch/servo.hpp), so a move
ends at the measured target rather than at a pulse count. Joint order is
shoulder, elbow, head; angles in degrees, speeds in degrees/second.

    with ArmMotion() as arm:
        arm.move_to([-5, 60, 10], speed=8)        # synchronized, then holds
        arm.move_to([0, 55, 0], duration=1.5)
        for pose, vel in planner:                 # behavior model, >= 4 Hz
            arm.stream_point(pose, vel, horizon=0.1)
        arm.relax()                               # drivers sleep

After a move the drivers stay powered and the loop keeps correcting (no sag)
until relax(), STOP, the hold timeout, or this session ends. Stalls are
detected from encoder progress; MOVE retries at most twice at half speed (the
shoulder first folds the elbow to cut its load) and reports an obstruction
instead of pushing again. Every call is recorded in the session JSONL.
"""
from __future__ import annotations
import json

from arm_client import ArmError, ArmSession, JOINTS, LIMITS, MOTOR, angles, finite, healthy

STATES = {0:'idle', 13:'moving', 14:'holding', 15:'recovering', 16:'stalled', 17:'streaming', 18:'fault'}
REASONS = {0:'none', 1:'complete', 2:'unsettled', 3:'lease', 4:'sensor', 5:'limit', 6:'timing',
           7:'stall', 8:'obstruction', 9:'halted', 10:'relaxed', 11:'hold_timeout'}
ACTIVE = (13, 15)
MARGIN = 2.0                       # firmware keeps targets this far inside hard limits
# Feedforward hints only, from the commissioning profile (shoulder, elbow, head).
DEFAULT_PPD = (146.0, 140.0, 143.1)


class ArmMotion(ArmSession):
    def __init__(self, *args, ppd=DEFAULT_PPD, max_speed=12.0, max_accel=40.0,
                 hold_timeout=60.0, tuck='auto', roll=None, **kwargs):
        """roll: configure_roll() keywords to use the head-tilt servo, or None to leave it alone."""
        super().__init__(*args, **kwargs)
        self.servo_config = dict(ppd=ppd, max_speed=max_speed, max_accel=max_accel,
                                 hold_timeout=hold_timeout, tuck=tuck)
        self.roll_config = roll
        self.roll_supported = False
        self.roll_range = None
        self.limits_speed = None

    def __enter__(self):
        super().__enter__()
        try:
            self.configure(**self.servo_config)
            if self.roll_config is not None:
                self.configure_roll(**self.roll_config)
        except BaseException:
            self.close(); raise
        return self

    def configure(self, ppd=DEFAULT_PPD, max_speed=12.0, max_accel=40.0, hold_timeout=60.0, tuck='auto'):
        """Per-joint feedforward/limits plus hold timeout and shoulder tuck.

        tuck: 'auto' folds the elbow opposite the shoulder's lean, a number
        folds it to that elbow angle, False disables the tuck retry.
        """
        speeds = max_speed if isinstance(max_speed, (list, tuple)) else (max_speed,)*3
        accels = max_accel if isinstance(max_accel, (list, tuple)) else (max_accel,)*3
        if len(ppd) != 3 or len(speeds) != 3 or len(accels) != 3:
            raise ValueError('Give one value or three (shoulder, elbow, head)')
        for j in range(3):
            p = finite(ppd[j], 20, 1000, 'Pulses/degree')
            v = finite(speeds[j], 0.5, 60, 'Maximum speed')
            a = finite(accels[j], 1, 500, 'Maximum acceleration')
            self._request(f'SERVO {MOTOR[j]} {round(p*1000)} {round(v*1000)} {round(a*1000)}')
        hold = round(finite(hold_timeout, 1, 600, 'Hold timeout')*1000)
        if tuck is False or tuck is None:
            mode, angle = 0, 0
        elif tuck == 'auto':
            mode, angle = 1, 0
        else:
            mode, angle = 2, round(finite(tuck, LIMITS[1][0]+MARGIN, LIMITS[1][1]-MARGIN, 'Elbow tuck angle')*1000)
        self._request(f'SERVOTIME {hold} {mode} {angle}')
        self.limits_speed = list(speeds)

    # Head tilt (J5 roll servo) ------------------------------------------------
    def configure_roll(self, center_us=1500, us_per_deg=2000/270, limits=(-45.0, 45.0), sign=1, max_speed=180.0):
        """Pulse mapping and limits for the head-tilt servo. Returns False (and roll() becomes a
        no-op) on firmware without the servo, so older firmware still runs everything else."""
        if not self.roll_status()['supported']:
            self.roll_supported = False
            return False
        lo, hi = (finite(v, -135, 135, 'Roll limit') for v in limits)
        if lo >= hi or sign not in (1, -1):
            raise ValueError('Roll range must be increasing and sign must be 1 or -1')
        center = round(finite(center_us, 500, 2500, 'Roll center'))
        per = round(finite(us_per_deg, 1, 20, 'Roll us/deg')*1000)
        vmax = round(finite(max_speed, 1, 1000, 'Roll speed')*1000)
        self._request(f'ROLLCONFIG {center} {per} {round(lo*1000)} {round(hi*1000)} {sign} {vmax}')
        self.roll_supported, self.roll_range = True, (lo, hi)
        return True

    def roll(self, angle, speed=None):
        """Tilt the head to angle (deg, clamped to the configured range), slewed at speed deg/s
        (None = the configured maximum). The servo holds there until roll_off() or the session ends."""
        if not self.roll_supported:
            return None
        lo, hi = self.roll_range
        angle = min(hi, max(lo, finite(angle, -1e6, 1e6, 'Roll angle')))
        speed_m = 0 if speed is None else round(finite(speed, 0, 1000, 'Roll speed')*1000)
        return self._request(f'ROLL {round(angle*1000)} {speed_m}')

    def roll_off(self):
        """Stop the servo pulses: the head tilt goes limp."""
        if self.roll_supported:
            return self._request('ROLLOFF')

    def roll_status(self):
        """{'supported', 'enabled', 'target', 'current' (deg), 'pulse_us', 'fault'}."""
        self._event('command', text='ROLLSTAT')
        s = self.transport.request('ROLLSTAT')
        self._event('roll', sample=s)
        if s.get('supported'):
            s = dict(s, target=s['target']/1000, current=s['current']/1000)
        return s

    # Reads -----------------------------------------------------------------
    def servo_status(self):
        """Detailed loop telemetry in joint order (also written to the recording)."""
        self._event('command', text='SERVOSTAT')
        s = self.transport.request('SERVOSTAT')  # does not replace self.sample
        self._event('servo', sample=s)
        axes = [s['axes'][m] for m in MOTOR]
        return {
            'state': STATES.get(s['state'], s['state']), 'reason': REASONS.get(s['reason'], s['reason']),
            'id': s['id'], 'retries': s['retries'], 'stalls': s['stalls'],
            'stalled_joint': JOINTS[MOTOR.index(s['stall_axis'])] if s['stall_axis'] >= 0 else None,
            'stall_angle': s['stall_angle']/1000 if s['stall_axis'] >= 0 else None,
            'planned_seconds': s['duration']/1000, 'elapsed_seconds': s['elapsed']/1000, 'scale': s['scale']/1000,
            'estimate': [a['estimate']/1000 for a in axes], 'reference': [a['reference']/1000 for a in axes],
            'goal': [a['goal']/1000 for a in axes], 'velocity': [a['velocity']/1000 for a in axes],
            'pulse_rate': [a['rate'] for a in axes],
            'ppd_estimate': [[a['ppd_forward']/1000, a['ppd_reverse']/1000] for a in axes],
        }

    def angles(self):
        return angles(self.status())

    # Motion ----------------------------------------------------------------
    def _pose(self, pose, what):
        if len(pose) != 3:
            raise ValueError('Pose requires shoulder, elbow, head')
        return [finite(v, LIMITS[j][0]+MARGIN, LIMITS[j][1]-MARGIN, f'{JOINTS[j]} {what}') for j, v in enumerate(pose)]

    def _movable(self):
        s = self.status()
        if not healthy(s) or s['configured'] != 7:
            raise ArmError('Fresh calibrated feedback required')
        if not s['enabled']:
            raise ArmError('Movement is locked; enable it first')
        return s

    def move_to(self, pose, duration=None, speed=None, timeout=None, telemetry_every=4):
        """Synchronized move; returns once holding, stalled, or on fault.

        duration (s) and speed (deg/s) are both optional; the firmware never
        goes faster than the configured limits and stretches the duration
        if needed. The result's 'ok' is True only for a settled, complete move.
        """
        pose = self._pose(pose, 'target')
        duration_ms = 0 if duration is None else round(finite(duration, 0.05, 60, 'Duration')*1000)
        speed_m = 0 if speed is None else round(finite(speed, 0.5, 60, 'Speed')*1000)
        s = self._movable()
        self.motion_id += 1
        mid = self.motion_id
        values = ' '.join(str(round(pose[j]*1000)) for j in MOTOR)
        s = self._request(f"MOVE {mid} {values} {duration_ms} {speed_m} {s['generation']}")
        began = self.clock()
        limit = timeout if timeout is not None else s['duration']/1000*2 + 15
        polls = 0
        try:
            while s['state'] in ACTIVE:
                if not healthy(s):
                    raise ArmError('Sensor feedback unavailable')
                if self.clock()-began > limit:
                    raise ArmError('Move exceeded its deadline')
                self.sleep(.025)
                s = self._request(f'HOLD {mid}')
                polls += 1
                if telemetry_every and polls % telemetry_every == 0:
                    self.servo_status()
        except BaseException:
            try:
                self._request('HALT')
            except Exception:
                pass
            raise
        return self._outcome(s, pose)

    def stream_point(self, pose, velocity=None, horizon=0.1):
        """Send one short-horizon setpoint: be at pose (moving at velocity)
        horizon seconds from now. Call at least every 0.25 s or the arm halts
        and holds. A stall makes the firmware grind through on the spot, slowly
        and powered; only if the joint won't move at all is the result's state
        'stalled', and the arm holds until halt().
        """
        pose = self._pose(pose, 'setpoint')
        top = self.limits_speed or [12.0]*3
        vel = [0.0]*3 if velocity is None else [finite(v, -top[j], top[j], f'{JOINTS[j]} velocity') for j, v in enumerate(velocity)]
        h = round(finite(horizon, 0.02, 2, 'Horizon')*1000)
        generation = (self.sample or {}).get('generation')
        if generation is None:
            generation = self._movable()['generation']
        self.motion_id += 1
        values = ' '.join(str(round(pose[j]*1000)) for j in MOTOR)
        speeds = ' '.join(str(round(vel[j]*1000)) for j in MOTOR)
        s = self._request(f'STREAM {self.motion_id} {values} {speeds} {h} {generation}')
        if s['state'] == 18:
            self._outcome(s, None)
        return {'state': STATES.get(s['state'], s['state']), 'angles': angles(s)}

    def scale(self, factor):
        """Live shared slowdown for the running move (0.05-1); path unchanged.
        This is the hook a learned stall-risk model would drive."""
        return self._request(f"SCALE {round(finite(factor, 0.05, 1, 'Scale')*1000)}")

    def halt(self):
        """Stop where it is and hold that pose powered."""
        return self._outcome(self._request('HALT'), None)

    def relax(self):
        """Stop and let every driver sleep (holding torque removed)."""
        return self.stop()

    def hold_for(self, seconds):
        """Keep the session (and powered hold) alive without moving."""
        seconds = finite(seconds, 0, 3600, 'Hold seconds')
        end = self.clock()+seconds
        s = self.status()
        while self.clock() < end:
            self.sleep(min(.5, max(0, end-self.clock())))
            s = self.status()
        return angles(s)

    def _outcome(self, s, pose):
        servo = self.servo_status()
        final = angles(s)
        result = {
            'ok': servo['state'] == 'holding' and servo['reason'] == 'complete',
            'state': servo['state'], 'reason': servo['reason'],
            'target': pose, 'final_angles': final,
            'error_deg': [final[j]-pose[j] for j in range(3)] if pose else None,
            'retries': servo['retries'], 'stalls': servo['stalls'],
            'stalled_joint': servo['stalled_joint'], 'stall_angle': servo['stall_angle'],
            'planned_seconds': servo['planned_seconds'], 'recording': str(self.output),
        }
        self._event('result', text=json.dumps(result, allow_nan=False))
        if servo['state'] == 'fault':
            raise ArmError(f"Closed-loop fault: {servo['reason']}")
        return result
