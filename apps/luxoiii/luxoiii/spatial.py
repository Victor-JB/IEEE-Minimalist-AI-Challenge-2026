"""Read-only monocular face localization. Coordinates: camera right/up/forward, metres.

Face width and horizontal FOV are assumptions, not a depth sensor. Body pose is
not inferred. Robot coordinates use calibrated encoders and the reported servo
command; a disabled servo's physical angle is unknown and needs a manual estimate.
"""
import math
from . import paths
from kinematics import fk


def estimate(snapshot, hfov=60.0, face_width=.16, pivot_fallback=0.0,
             camera_offset=.05, camera_pitch=0.0, camera_yaw=0.0):
    if not (20 <= hfov <= 150 and .08 <= face_width <= .30 and
            -.2 <= camera_offset <= .3 and -90 <= camera_pitch <= 90 and
            -180 <= camera_yaw <= 180 and -90 <= pivot_fallback <= 90):
        raise ValueError('Camera settings outside supported range')
    width, height = snapshot.get('size', [640, 480])
    if not (0 < width <= 8192 and 0 < height <= 8192):
        raise ValueError('Invalid image size')
    fresh = 0 <= snapshot.get('age_s', math.inf) < 2
    f = width / (2 * math.tan(math.radians(hfov) / 2))
    result = {**snapshot, 'people': [], 'camera': None, 'fresh': fresh,
              'hfov': hfov, 'vfov': math.degrees(2 * math.atan(height / (2*f))),
              'assumed_face_width_m': face_width, 'base_yaw_source': 'fixed zero reference'}
    if not fresh:
        result['error'] = snapshot.get('error') or 'Camera data is stale'
        return result
    joints = snapshot.get('joints', [])
    valid = len(joints) == 3 and all(isinstance(j, dict) and
        isinstance(j.get('angle'), (int, float)) and math.isfinite(j['angle']) and
        0 <= j.get('age_ms', math.inf) <= 100 and j.get('healthy') for j in joints)
    pivot = snapshot.get('pivot') or {}
    reported = pivot.get('current')
    known_pivot = bool(pivot.get('enabled')) and not pivot.get('fault') and isinstance(reported, (int, float)) and math.isfinite(reported)
    roll = reported if known_pivot else pivot_fallback
    result['pivot_source'] = 'servo command (no encoder)' if known_pivot else 'manual estimate (servo released)'
    if valid:
        h = fk([j['angle'] for j in joints], roll)
        position = [h.pivot[i] + camera_offset*h.axis[i] for i in range(3)]
        # Mount yaw about image up, then mount pitch about image right.
        y, p = math.radians(camera_yaw), math.radians(camera_pitch)
        d = [math.cos(y)*h.axis[i] + math.sin(y)*h.right[i] for i in range(3)]
        right = [math.cos(y)*h.right[i] - math.sin(y)*h.axis[i] for i in range(3)]
        forward = [math.cos(p)*d[i] + math.sin(p)*h.up[i] for i in range(3)]
        up = [math.cos(p)*h.up[i] - math.sin(p)*d[i] for i in range(3)]
        result['camera'] = dict(position=position, right=right, up=up, forward=forward,
                                roll=roll, joints=[j['angle'] for j in joints],
                                chain=[(0,0,.049), h.elbow, h.wrist, h.pivot, position])
    for box in snapshot.get('detections', []):
        x, y, w, h = [box[k] for k in ('x', 'y', 'w', 'h')]
        if not all(math.isfinite(v) for v in (x,y,w,h)) or min(w,h) < 12:
            continue
        # A cropped face cannot supply a usable apparent width.
        if x <= 0 or y <= 0 or x+w >= width or y+h >= height:
            continue
        z = f * face_width / w
        if not .15 <= z <= 5:
            continue
        point = [(x+w/2-width/2)*z/f, -(y+h/2-height/2)*z/f, z]
        person = dict(camera=point, depth_m=z, range_m=math.dist(point, (0,0,0)),
                      score=box['score'], bbox=[x,y,w,h])
        if result['camera']:
            c = result['camera']
            person['world'] = [c['position'][i] + point[0]*c['right'][i] +
                               point[1]*c['up'][i] + point[2]*c['forward'][i] for i in range(3)]
        result['people'].append(person)
    return result


def read_observer(url, query):
    """Fetch one snapshot; count network time toward its age and fail closed."""
    import json
    import time
    from urllib.request import urlopen
    try:
        if not url:
            raise ConnectionError('Live observer is not connected')
        settings = {k: float(query[k][0]) for k in
            ('hfov','face_width','pivot_fallback','camera_offset','camera_pitch','camera_yaw') if k in query}
        started = time.monotonic()
        with urlopen(url.rstrip('/') + '/spatial', timeout=2) as response:
            snapshot = json.loads(response.read(2_000_000))
        snapshot['age_s'] = snapshot.get('age_s', 1e9) + time.monotonic() - started
        return estimate(snapshot, **settings)
    except Exception as exc:
        return dict(error=str(exc), fresh=False, people=[], camera=None)
