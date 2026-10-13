# luxoiii spatial observer

The Mac web view at port 8082 combines the full CAD assembly with read-only encoder readings,
head-pivot command telemetry, a camera frustum and approximate face positions. Detection runs
on the UNO Q using the existing quantized face model. The default observer is read-only. With `--hybrid`, the UI can explicitly activate only the physical
head pivot and D7 lamp; failed stepper joints are animated in the browser. All outputs start disabled.

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
`apps/lumen/.venv/bin/python`. Copy all four Python files from this folder together if running from `/tmp`; it imports the
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
60° horizontal FOV, 16 cm face width, 5 cm along the shade, zero mount rotation and zero released
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
and a released servo uses the visible manual estimate. The geometric hand estimates select a desk target for the hybrid demo. Physical outputs are separately
gated using encoder telemetry and the fixed arm’s reach; they do not command stepper movement.

Stopping the observer, SSH tunnel, or preview does not stop or alter the existing desk process.


## Gestures and physical outputs

- Point downward and hold for 0.4 s (at least two detections): an estimated index-finger ray intersects
  the desk plane. Only reachable points 20–65 cm forward and within 30 cm sideways are accepted.
  The display marks that point and the simulated arm acknowledges, then aims using shared arm IK.
  A held point that misses the desk is retried; the UI asks for a reachable downward point.
- Thumbs up/down, held 0.4 s: brighter/closer or dimmer/further, repeating at most once a second.
  Simulation steps along the beam; real hardware changes brightness within 10–80% perceptual level.
- Open palm, held one second: latch Stop, turn off demo outputs, and freeze the simulated arm.
  Choose **Resume gestures** to clear the latch, then separately enable physical outputs if wanted.
- Slow wave: two wrist reversals within 2.5 s greet back in the attentive state. The simulated arm
  and enabled physical pivot make a small wave. A stationary held open palm always wins as Stop.

**Enable real lamp + pivot** grants the sidecar a quiet-arm session. The driver’s explicit allowlist
contains claim/status/release and pivot config/move/off only. It never sends stepper configuration,
move, stream, jog, or power-enable commands. It refuses an already owned or awake arm and bounds
pivot commands to ±10° at 20°/s. The existing desk remains the camera owner.

The physical lamp is inhibited unless pitch encoders are healthy, the shade points downward
(110–170° cumulative pitch), no face is in view, the commanded pivot has settled, and the selected
spot is within 18 cm of the desk intersection reachable by the real pivot. The fixed arm cannot
follow all targets reachable by the simulation. The current physical pose is above the desk, so
its lamp remains off until the supported head is positioned appropriately.

Browser heartbeat expiry (3 s), stale camera/encoders, Stop, or a bridge failure disables outputs.
The lamp refresh is a positive-level write followed immediately by an 800 ms fade-to-zero write,
renewed at 10 Hz. This provides ordinary process-loss fadeout, but is **not an atomic firmware
watchdog**: failure between those writes can leave the level set. An MCU light-lease protocol would
be required to guarantee cutoff in that narrow case; no firmware was reflashed for this demo.

Targeting uses the saved joint calibration and fixed assumptions: 60° HFOV, 8 cm palm width,
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
desk targets use the simulated IK pose. The camera and amber physical skeleton always use telemetry.
**Show emotions** cycles the authored clips without creating AI events or physical commands. The
browser samples sparse per-joint curves at display rate; the lamp’s unsupported wrist-yaw channel
is omitted. **People + robot** fits the wider scene; **3D** returns to a closer arm view.

With physical outputs enabled, **Swivel −8°**, **Center swivel**, and **Swivel +8°** command the
working head servo. Manual swivel inhibits the lamp. **Automatic swivel** restores small head tilts
in response to a visible person/hand and the gesture response. This is an expressive tilt, not
calibrated gaze tracking. Keep the preview open to renew its output heartbeat.

Camera capture now retries after USB disconnection while a viewer remains connected. A capture
error clears its cached image and FPS; a timeout never retransmits the previous JPEG. The observer
also refuses to timestamp repeated identical JPEGs as fresh, covering older desk deployments.
