# luxoiii spatial observer

The Mac web view at port 8082 combines the full CAD assembly with read-only encoder readings,
head-pivot command telemetry, a camera frustum and approximate face positions. Detection runs
on the UNO Q using the existing quantized face model. No motor/lamp commands are issued.

The observer shares the standalone desk's MJPEG feed at port 8081 (does not open the USB camera
again). It holds one frame in memory and runs face detection with two threads at up to 2 Hz.
Images and detections are not saved. The observer is a temporary process, not a boot service.

From the repo on the board, while the standalone desk is running:

```sh
apps/luxoiii/.venv/bin/python tools/spatial/observe.py --repo "$PWD"
```

For the existing, unmigrated board deployment, use `--package lumen` and its existing
`apps/lumen/.venv/bin/python`. The observer may be copied separately to `/tmp`; it imports the
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
measured joints** displays the physical pitch angles; **Play animation** moves only the model.
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
and a released servo uses the visible manual estimate. Estimates never drive physical actuators.

Stopping the observer, SSH tunnel, or preview does not stop or alter the existing desk process.
