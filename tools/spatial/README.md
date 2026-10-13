# luxoiii spatial observer

The Mac web view at port 8082 combines the full CAD assembly with read-only encoder readings,
head-pivot command telemetry, a camera frustum and approximate face positions. Detection runs
on the UNO Q using the existing quantized face model. The default observer is read-only. With `--hybrid`, the UI can explicitly activate only the physical
head pivot for subject tracking; the D7 lamp stays off. Failed stepper joints are animated in the browser. All outputs start disabled.

The observer shares the standalone desk's MJPEG feed at port 8081 (does not open the USB camera
again). It holds one frame in memory and runs face detection with two threads at a bounded active loop. A motion gate skips neural models when nobody is present, with a three-second
probe to recover stationary arrivals. Active hand tracking reuses landmark crops; UI counters expose
actual model calls, gated frames and measured timing.
Images and detections are not saved. The observer is a temporary process, not a boot service.

From the repo on the board, while the standalone desk is running:

```sh
apps/luxoiii/.venv/bin/python tools/spatial/observe.py --repo "$PWD" --hybrid
```

For the existing, unmigrated board deployment, use `--package lumen` and its existing
`apps/lumen/.venv/bin/python`. Copy all five Python files from this folder together if running from `/tmp`; it imports the
installed detector without replacing the running application.

On the Mac, in one terminal:

```sh
ssh -N -L 18083:127.0.0.1:8083 arduino@100.78.199.105
```

In another, from the repo root:

```sh
PYTHONPATH=apps/luxoiii python3 -m luxoiii.preview --observer-url http://127.0.0.1:18083
```

Open http://127.0.0.1:8082/. Choose **People + robot** to fit face markers into view. **Follow
measured joints** displays the physical pitch angles; **Show emotions** moves only the model.
The amber camera and physical joint chain stay tied to hardware. **Example person** is an
explicitly labeled illustration for verifying the geometry when nobody is in the frame.

Calibration controls set horizontal field of view, assumed face width, camera offset along the
shade, mount yaw/pitch, and the released servo's manually estimated angle. Defaults are assumptions:
100° horizontal FOV, 16 cm face width, 5 cm along the shade, zero mount rotation and zero released
pivot. The user confirmed the camera moves with the pivot. Its exact mounting offset/orientation
and lens intrinsics still need measurement. Base yaw is a fixed zero reference, not a sensor.

Depth = focal length in pixels × assumed face width / face-box width. This is monocular apparent-size
localization, not a depth sensor, SLAM, skeletal pose, or identity tracking. Turning/occluding a face
and differences in face width can cause large errors. Cropped/tiny faces and estimates outside
0.15–5 m optical depth are omitted. Camera coordinates are right/up/forward. Robot coordinates are
forward/left/up; the browser maps them to right/up/back. Joint geometry is shared with `kinematics.py`.

Raw encoder samples use the saved October 11 calibration, applied off-board from unclaimed status
reads. Read-only status may return EACCES/EBUSY alongside valid samples; these are not motor claims.
Unhealthy or >100 ms old encoder samples never produce world coordinates. Frames older than two
seconds clear face markers. The servo has no encoder: powered readings are commanded angles,
and a released servo uses the visible manual estimate. The geometric hand estimates select a desk target for the hybrid demo. The real head tracks subjects independently; no gesture or digital animation commands its position.

Stopping the observer, SSH tunnel, or preview does not stop or alter the existing desk process.


## Social behavior, gestures and subject tracking

The real swivel is exclusively an image-based subject tracker. **Enable subject tracking** grants
it a quiet-arm session. Faces take priority over hands; among multiple faces it follows the nearest
previous image-space subject. After a one-second face-loss grace period, a visible palm can become
the target. It corrects horizontal camera error using current servo command telemetry and a 100°
HFOV assumption. A 3.5% image-width deadband and 12°/s rate limit reduce jitter. Travel is ±45°;
vertical centering is unavailable with the broken pitch joints. Losing a subject holds the last
angle; the tracker does not scan or play emotional motions. This is not identity recognition.

The D7 lamp stays off because the camera follows people. Lighting gestures affect the digital
scene only. The hardware RPC allowlist contains claim/status/release and pivot config/move/off;
no stepper configuration, move, stream, jog or power-enable commands are sent. An owned or awake
arm is refused. Browser heartbeat expiry (3 s), stale camera/encoders and Stop disable outputs.

The digital arm uses the original authored clips, with gestures taking priority over social loops:

- After ten seconds without a face or hand: idle breathing and occasional room-scanning loops.
- First arrival: perk-up greeting or double-take. Another person joining for at least 0.8 s elicits
  a curious greeting. Detection counts are not identity tracking.
- After eight seconds without an accepted gesture: curious tilts, leaning, a playful wave/bounce,
  and attentive breathing while someone remains visible.
- Point downward for 0.4 s: intersect the estimated finger ray with the simulated desk. Reachable
  points 20–65 cm forward and within 30 cm sideways receive a nod, an IK aiming pose, and a visible
  simulated light cone and target ring. Invalid points produce a shrug.
- Thumbs up/down for 0.4 s: simulated brighter/closer or dimmer/further, at most once per second.
- A moving palm can wave. A **still** open palm held for one second latches Stop, freezes the
  digital arm and releases physical tracking. Choose **Resume gestures**, then enable tracking.

Targeting uses the saved joint calibration and fixed assumptions: 100° HFOV, 8 cm palm width,
5 cm axial camera mount, no mount rotation, zero base yaw, and zero released pivot. The user
confirmed the camera moves with the head pivot. `pointing.py` converts relative landmark depth
into estimated camera coordinates before intersecting the finger ray with the desk. Hand-world
landmarks are not treated as absolute room coordinates. Oblique palms, occlusion, calibration
error and monocular depth uncertainty can shift the target. View calibration controls currently
adjust visualization only; change the board defaults in `pointing.py` too when calibrating.

This is an explicit desk-gesture demo beside the regular application. It does not implement book
recognition, replace the regular behavior machine, or establish competition efficiency scores.


## Physical pose and expressive motion

The always-visible angle row shows all three calibrated encoder readings and the head servo’s
command estimate (there is no swivel encoder). **Follow measured joints** puts the complete CAD
model at that pose. In **AI-driven simulated arm**, the measured pose anchors the original authored
motion curves: perk-up on arrival, curious tilts and breathing while attentive, a wave in response,
a nod for accepted pointing/brightness changes, and a shrug for an unreachable desk point. Accepted
desk targets use the simulated IK pose with gentle loops. The digital head roll does not copy the
physical tracker; pitch encoder readings provide its neutral pose. The camera and amber physical skeleton always use telemetry.
**Show emotions** cycles the authored clips without creating AI events or physical commands. The
browser samples sparse per-joint curves at display rate; the lamp’s unsupported wrist-yaw channel
is omitted. **People + robot** fits the wider scene; **3D** returns to a closer arm view.

Camera capture now retries after USB disconnection while a viewer remains connected. A capture
error clears its cached image and FPS; a timeout never retransmits the previous JPEG. The observer
also refuses to timestamp repeated identical JPEGs as fresh, covering older desk deployments.
