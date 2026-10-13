# luxoiii main app · board observer

The Mac web view at port 8082 combines the full CAD assembly with read-only encoder readings,
head-pivot command telemetry, a camera frustum and approximate face positions. Detection runs
on the UNO Q using the existing quantized face model. The default observer is read-only. With `--hybrid`, the UI can explicitly activate only the physical
head pivot for subject tracking and D7 under a face/downward-beam gate. Failed stepper joints are animated in the browser. The observer starts with outputs disabled; the preview enables tracking and automatic lighting once camera and encoders are fresh.

The observer shares the standalone desk's MJPEG feed at port 8081 (does not open the USB camera
again). It holds one frame in memory and runs face detection with four threads at a bounded active loop. In hybrid mode, every processed frame runs face detection, including quiet scenes: an unchecked frame never counts as face-free. A motion gate can still skip the hand model, with a three-second
probe to recover stationary arrivals. Active hand tracking reuses landmark crops; UI counters expose
actual model calls, gated frames and measured timing.
Up to two hands are detected. Pointing and command gestures have independent hold timers:
one hand can steer the desk light while the other gives Close/Far. The pointer follows its
previous wrist position when detector results reorder; a distant replacement needs a fresh hold.
This is short-term spatial association, not persistent hand/person identity. Crossing or occluding
hands can confuse it. Two opposite thumbs cancel. Keep both hands fully visible and separated.
Images and detections are not saved. The main app deployment installs this as `luxoiii-observer.service`; the foreground command below is also supported.

From the repo on the board, while the standalone desk is running:

```sh
apps/luxoiii/.venv/bin/python tools/spatial/observe.py --repo "$PWD" --hybrid
```

For the existing, unmigrated board deployment, use `--package lumen` and its existing
`apps/lumen/.venv/bin/python`. Copy all Python files from this folder together if running from `/tmp`; it imports the
installed detector without replacing the running application.

On the Mac, in one terminal:

```sh
ssh -N -L 18083:127.0.0.1:8083 arduino@100.78.199.105
```

In another, from the repo root:

```sh
PYTHONPATH=apps/luxoiii python3 -m luxoiii --observer-url http://127.0.0.1:18083
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

The real swivel is exclusively an image-based subject tracker. The preview grants
it a quiet-arm session by default once observations are fresh. Faces take priority over hands; among multiple faces it follows the nearest
previous image-space subject. After a one-second face-loss grace period, a visible palm can become
the target. It corrects horizontal camera error using current servo command telemetry and a 100°
HFOV assumption. The middle 40% of the image is a hold zone; tracking resumes outside the
middle 52% after two consistent observations. PD velocity control (Kp 1.8, Kd 0.35) brakes
closing error, filters its derivative and never integrates error. Corrections are capped at 6°
per observation and 45% of the remaining error outside the hold zone. A fresh servo command
estimate after inference compensates for travel since frame capture, avoiding corrections
based on an old camera heading. Reversals need confirmation. Firmware slew remains capped
at 60°/s, with smaller steps near the zone. Travel is ±45°;
vertical centering is unavailable with the broken pitch joints. Losing a subject holds the last
angle; the tracker does not scan or play emotional motions. This is not identity recognition.

The D7 lamp switches fully on (digital HIGH, no PWM) only after at least three distinct face-checked frames and one
second with no detected faces, and only when the entire estimated beam is below horizontal.
Fresh physical encoder and swivel readings are taken separately in each output cycle, so a
simulated pose or an old camera-time joint reading cannot authorize light. The current swivel,
its command in flight and the next tracking target are checked before moving. The aperture model
uses a half-angle of atan(10.5 cm / (14 cm − 5 cm)) ≈ 49.4°, with 5° clearance to turn on and
2° to stay on. This assumes a level base, the saved encoder calibration, and the modeled LED
position. The real beam profile/LED position are not measured, and the swivel is a command
estimate rather than an encoder measurement. These are geometric/model limits, not a guarantee
that no light can reach an unseen face. The page exposes the estimated upper beam edge.

A detected face, a failed/skipped/older-than-one-second face check, an unsafe orientation,
unavailable pose, Stop or stale camera turns D7 off. Every nonzero light command includes an
800 ms MCU lease; if the observer crashes or its updates stop, the independent MCU tick turns
the light off. Older firmware without lease support refuses activation. The updated firmware
is required; its two-argument manual light commands also switch fully on/off, without PWM.

Lighting gestures affect the digital scene only. The hardware motion RPC allowlist contains claim/status/release and pivot config/move/off;
no stepper configuration, move, stream, jog or power-enable commands are sent. An owned or awake
arm is refused. Browser heartbeat expiry (3 s), stale camera/encoders and Stop disable outputs.

The digital arm uses the full-model branch’s absolute choreography with blended additive expressions. Gestures take priority over social loops:

- After ten seconds without a face or hand: settle into the original sleep pose, breathe and occasionally stretch or look around.
- First arrival: ease into the attentive pose over 1.4 s, then give one moderate 17° head nod
  with a short pause and a slower return. The body stays composed during the nod.
  Another person joining for at least 0.8 s elicits
  a look-around greeting. The digital arm shares attention across visible face positions every 3.5–5.5 s; proximity matching stabilizes detection ordering, without identity recognition.
- After eight seconds without an accepted gesture: curious tilts, leaning, happy bounces mixed with
  dance/sway, then a pause for a response. Unanswered invitations grow farther apart. A hand gesture
  cancels the invitation immediately; stale events are never queued or replayed.
- Extend only the index finger for 0.4 s: preview a desk target, then acknowledge and light it.
  A valid estimated 3D finger ray selects a spot 20–65 cm forward and within 30 cm sideways.
  If the ray misses (common with monocular depth and the side-facing camera), a **Virtual desk
  pointer** maps fingertip image position to 25–65 cm forward and ±30 cm sideways. Image top/bottom
  controls far/near; image right maps to robot right. This fallback selects a simulated spot, not
  a measured real-world point. A blue ring previews selection in the scene and at the fingertip.
  After that initial hold, every valid pointing observation updates the target, including small
  movements. A 160 ms time-based filter reduces jitter; the browser smooths the arm each animation
  frame and polls at 150 ms intervals without overlapping requests. Moving the finger does not
  restart acknowledgments. Once a drag uses the virtual cursor, it keeps that mapping until release
  instead of jumping back to a different ray target. Lowering/losing the hand holds the last spot;
  reacquisition needs the brief hold again. No target motion is extrapolated from missing frames.
- Thumbs up for 0.4 s: **Far**, moving away from the spot. Thumbs down: **Close**, moving toward
  it. Steps request 3 cm at most once per second, within the model's reachable range and a ±24 cm
  adjustment limit. Both keep the beam aimed at the spot; the chosen distance offset carries into
  subsequent pointing. Lamp power stays constant, with natural distance falloff in the rendering.
- Show an open palm for 0.4 s (at least two detections): the simulated arm greets back, in any
  active state. No lateral movement is required. A held palm triggers once; lower and show it again
  to repeat, with a three-second cooldown. Palm detections never pause gestures or tracking.
- Gestures stay active by default. Only the on-screen **Stop & lamp off** button latches Stop;
  **Resume gestures** restores social interaction and the saved tracking preference.

Targeting uses the saved joint calibration and fixed assumptions: 100° HFOV, 8 cm palm width,
5 cm axial camera mount, no mount rotation, zero base yaw, and zero released pivot. The user
confirmed the camera moves with the head pivot. `pointing.py` converts relative landmark depth
into estimated camera coordinates before intersecting the finger ray with the desk. A failed ray
falls back to an explicitly labeled 2D virtual cursor; it is not silently clamped or called a
physical hit. The digital base yaws toward the chosen target, and a raised wrist plus head pitch
puts the actual simulated beam axis through it without the real arm’s SAFE_RANGES. Close/Far
adjustments preserve that yaw and target while moving along the simulated beam. Hand-world
landmarks are not treated as absolute room coordinates. Oblique palms, occlusion, calibration
error and monocular depth uncertainty can shift the target. View calibration controls currently
adjust visualization only; change the board defaults in `pointing.py` too when calibrating.

This is an explicit desk-gesture demo beside the regular application. It does not implement book
recognition, replace the regular behavior machine, or establish competition efficiency scores.


## Physical pose and expressive motion

The always-visible angle row shows all three calibrated encoder readings and the head servo’s
command estimate (there is no swivel encoder). **Follow measured joints** puts the complete CAD
model at that pose. **AI-driven simulated arm** uses independent full-body poses from
`full-model-animations@ef1b0d2057c5e449a0e02638a1ff7d5b58dd3cbd`, not a measured-pose baseline.
The camera and amber physical skeleton always use telemetry. Physical swivel limits are unchanged.

`arm/clips/full-model.json` contains 11 absolute action timelines extracted from `lamp_demo.py`
and six additive oscillations from `elegnt_demo.py`: breathe, nod, shake, bounce, dance and sway.
The original full show retains all source poses, minimum-jerk timing and per-joint stagger.
Short actions select passages for interaction; the mixer adds breathing, gentle spatial attention,
and layered responses with smooth entry/return transitions. Browser oscillator amplitudes use
source expression values without physical stepper caps. This is a kinematic animation adaptation,
not MuJoCo physics or a collision-validated combination of the original movements.
Regenerate with `python3 scripts/import_full_model_animations.py`; the importer parses literals
and numeric expressions without executing branch code, and records source hashes.

The scene includes a solid wood desktop lit by a per-fragment spotlight, with soft cone edges,
angle/distance falloff and an emissive shade diffuser. Its origin and direction follow the
animated model’s actual five-joint FK, rather than drawing a cone to the selected marker.
Thumb gestures change its distance from the target. No wireframe light cone is drawn; the target marker
and pointing cursor remain visible. This is direct diffuse lighting, without shadow maps.
**Preview desk light** is an explicitly labeled, digital-only lighting preview; it does not
create a detection or request D7. Physical automatic lighting uses its own face/pose gate in
every display mode. AI mode restores live gesture lighting.

Pointing/illumination reserves the desk aiming pose. The initial acknowledgment settles before
continuous following; active pointing suppresses nods and return blends so the beam follows the
finger. Breathing, gaze and unsolicited body motions yield to this task.
**Show emotions** cycles full-body animations. **Try an animation mix** lets you combine any
source action (including the original full show) with a nod, wag, bounce, or dance + sway.
These previews are digital only; **AI-driven simulated arm** returns to interaction.
**People + robot** fits the wider scene; **3D** returns to a closer arm view.

Camera capture now retries after USB disconnection while a viewer remains connected. A capture
error clears its cached image and FPS; a timeout never retransmits the previous JPEG. The observer
also refuses to timestamp repeated identical JPEGs as fresh, covering older desk deployments.


Automatic defaults: the preview opens in AI animation mode and enables subject tracking and gated lighting after
fresh camera and encoder data arrive. A manual tracking-off selection is saved in this browser.
Reloading never clears a latched Stop; **Resume gestures** resumes the social loops and restores
tracking when the saved preference is on. Stale data or heartbeat expiry still disables hardware;
a failed activation is not retried repeatedly. The camera image shows the center hold zone.

## Measured inference settings

The default runs each w8a8 model sequentially with four LiteRT CPU threads and
one OpenCV preprocessing thread. The thread count was measured on the UNO Q;
face and hand thresholds, input resolution and model weights are unchanged.
Use `--inference-threads 2 --parallel-inference` only to reproduce the concurrent
ablation, or `--inference-threads 2 --serial-inference` for the prior schedule.

Face heatmap peaks are selected directly in uint8, with a 256-entry sigmoid
lookup and dequantization of candidate boxes only. Hand crops expire after a
camera/inference gap. The conservative second-hand search remains enabled on
every frame while only one hand is tracked. Experimental palm skipping is not
the live default because its temporal accuracy audit lost several matches.

[Evaluation tools and dataset methodology](../efficiency/README.md) and
[measured scorecard](../../reports/efficiency/README.md) document the before/after
results, identical still-image predictions, and remaining accuracy limitations.

Gesture recognition now uses a 21 KB trained landmark classifier after the same
hand model. It recognizes foreshortened pointing and rejects non-command hand
poses without another image-model pass. The pipeline reports `landmark-mlp-v1`;
if the artifact cannot load, it warns and reports `legacy-rules`. Deploy
`gestures.py`, `gesture_features.py`, and `models/gesture_landmarks_v1.npz`
together. Landmark presence stays at 0.65 and command confidence is 0.5.
See the [gesture evaluation](../../reports/gestures/README.md) for public-photo
accuracy, missed detections, and measured board latency.
