# Hardware talking points

For the System Design slide (20 pts) and the Edge Performance / power story (20 pts). Sources are linked so
whoever presents can answer follow-ups.

## 1. The arm in one breath

A 3-joint lamp arm (shoulder, elbow, head) driven by NEMA-17 steppers through **16:1 inside-out cycloidal
gearboxes**, with an **absolute magnetic encoder on every joint output**, closed-loop on the STM32 half of the
UNO Q, while the Linux half does vision. Visual: `arm_mujoco/arm_parts.png`, `arm_mujoco/drive_parts.png`.

| Joint | Motor | Reduction | Torque at joint | Range |
|---|---|---|---|---|
| Shoulder | 17HS4401 | cycloid 16:1 | 6.4 N·m (stall) | −45…+38° |
| Elbow | 17HS4401 | cycloid 16:1 | 6.4 N·m | −72…+79° |
| Head | 17HS4023 "pancake" | cycloid 16:1 | 2.48 N·m | −80…+87° |

Design choices worth saying out loud:
- **Mass placement.** Shoulder and elbow motors both live in the first link, close to the base; the head uses
  the thin pancake motor. Less weight far out = less torque and inertia for every emote.
- **Sensors on the joint output, not the motor shaft.** The AS5600s read the actual joint angle, so they see
  backlash, gear flex and lost steps, and they're absolute at power-up (no homing dance). Three sensors share
  one I²C bus through a PCA9548A mux (they all have the same address). (`arm_mujoco/README.md` "Sensing")
- **Closed loop on the microcontroller.** Every 1 ms the STM32 corrects each joint toward the trajectory using
  the encoder. Result: worst endpoint error **0.30° at 60 °/s**, 54/54 bench moves with 0 stalls.
  (`apps/stepper_desk/CLOSED_LOOP.md`)
- **Safety lives in firmware, not Python.** Joint limits, stale-sensor stop, stall detection, and a 300 ms
  lease: if Linux crashes or the link drops, the arm stops on its own.

## 2. Why an inside-out ("inverted") cycloidal gearbox

What it is (one sentence for the slide): the motor spins an eccentric cam; three printed cycloid plates
**orbit 0.8 mm without spinning**, and their 17 lobes roll around **16 steel pins fixed to the next link**,
so the link turns 1/16 of a turn per motor turn. "Inside-out" = the pin ring is the output and rotates, instead of
being the fixed housing as in a classic cycloid. That removes the separate output mechanism (no output-pin
holes / Oldham coupling): the next link's axle *is* the output.

Justification, in the order judges care about:

1. **Torque from a small motor.** A ~0.4 N·m NEMA-17 becomes ~6.4 N·m at the joint in a single stage. The
   holding analysis says the shoulder needs up to 2.7 N·m in the worst pose (`sim/view.py --torque`), so a
   direct-drive stepper or hobby servo can't hold the arm.
2. **Compact and coaxial.** 16:1 in one flat stage that fits inside the link shell, on the joint axis. A
   planetary needs two stages for the same ratio; a belt needs space and tensioning.
3. **Low backlash for a printed gearbox.** Many lobes share the load (rolling contact). Measured reversal slack
   on the real arm: shoulder ~0.4°, elbow ~0.5°, head ~0.7–0.8°, and the encoder loop absorbs it at endpoints.
4. **Three plates, 120° apart.** Balances the orbiting mass (less vibration) and means the drive never sits at
   a dead point.
5. **Hard to back-drive.** The arm holds its pose with the motors unpowered. This is what makes the sleep pin
   (next section) possible: **the gearbox choice is a power-saving choice.**
6. **Quiet and printable.** Rolling contact is quiet enough for a desk; everything except bearings, pins
   and motors is PLA, so it's cheap and iterable (v29 of the arm).
7. **Why not servos?** The project started as a 6-servo design (`apps/expressive_robot`, legacy). Hobby servos
   jitter, have weak torque at this size, and hide their position loop, so you can't do smooth, slow,
   expressive motion or read true joint angles.

Honest caveats (have answers ready):
- Printed lobes wear; backlash may grow. Encoder-on-output means accuracy degrades gracefully.
- 16:1 caps speed: the closed loop tops out around 84 °/s at the joint (12,000 pulses/s cap). That's plenty for
  expressive motion, where the clips run at 22–30 °/s anyway.
- Measured limit: the shoulder lifts only ~1.27 N·m while moving (stepper *running* torque is well below
  *holding* torque). We found that on hardware, fitted it into the sim, and now check clips against it before
  they run on the arm. See section 4.

## 3. The sleep pin: power savings

**Mechanism.** All three DRV8825 SLEEP inputs share one GPIO (D8). The firmware boots asleep and wakes the
drivers only for an accepted move (3 ms wake delay before any step). When the last joint stops, for any reason
(done, limit, watchdog, fault), it sleeps all three. Encoder polling continues while asleep, so the robot still
knows its pose. (`apps/stepper_desk/README.md` "Automatic driver sleep on D8", `sketch/power.hpp`)

**Why it saves power.** A stepper holding position burns full coil current (I²R) while doing no work. Asleep,
the DRV8825 cuts coil current to zero and draws µA.

Back-of-envelope (replace with a measurement before presenting):

```
P_hold per motor ≈ I_limit² · R_phase   (two phases at 0.707·I in the full-step position)
example: 1.0 A limit, 1.5 Ω (17HS4401)  ≈ 1.5 W per motor
three motors + driver losses            ≈ 4–5 W held  →  ≈ 0 W asleep
```

That is likely **more than the whole UNO Q + camera draw**, so for a lamp that sits still most of the day,
sleep is the single biggest power lever in the system. Frame it with duty cycle: if the arm moves 10% of
the time, sleep removes ~90% of motor energy.

Side benefits to mention:
- **Thermal.** The evening slowdown in `CLOSED_LOOP.md` (elbow and head losing torque after hours powered) points
  to heat in the motors or drivers. Sleeping between moves keeps them cool, and cool steppers keep their torque.
- **Safety.** Asleep = no torque, so a person can't get pinched by an idle arm.
- **Cost.** One GPIO and one wire.

**Measure it (10 minutes, high value for the scorecard):** read the 20 V VMOT bench-supply current with the
arm (a) holding a pose awake (Sleep & lock off, PAUSE / held pose) and (b) asleep at the same pose. Report
W awake-hold vs W asleep, and the % of a 5-minute scripted demo spent asleep (the firmware's `drivers_awake`
telemetry gives the duty cycle). Add both rows to `perf/RESULTS.md`.

**Tie it to the AI story:** the same idea runs through the whole stack. The motion gate idles the neural
nets, the state machine idles the camera rate, the sleep pin idles the motors. **Nothing runs unless the lamp
is paying attention.**

## 4. Simulation of the current arm (rerun 2026-10-11, lamp-style motion)

`python sim/lamp_show.py` plays "a lamp's day" on `sim/stepper_desk.toml` (CAD geometry and masses, current
limits) through the exact clip conversion the board uses: rest on the desk, notice you and lift the
head, curious tilt, nod, bounce, back to the desk, point, celebrate, refuse, shrug, sigh, stretch, doze off, sleep.
Outputs: `docs/presentation/stepper_desk_show.mp4` and `stepper_desk_show.png`.

| | shoulder | elbow | head |
|---|---|---|---|
| Peak speed (°/s) | 13.3 | 20.4 | 26.9 |
| Sim tracking error (max) | 0.17° | 0.48° | 0.41° |
| Peak required torque (N·m, inverse dynamics) | 0.72 | 1.22 | 0.67 |
| Limit | 2.3 hold / **1.27 lift** | 2.0 | 1.0 |

All 19 clips play at time_scale 1.00 (no slowdown), and the shoulder never comes near its lifting limit.

What changed, and why it's a talking point ("we designed the motion for the machine we have"):
- **Posture:** the old rest pose aimed the shade 33° *up*, so every animation looked at the ceiling. Now the
  lamp rests looking at the desk (shade 20° below level), lifts its head to look at you when it notices you,
  and crouches with the shade hanging to sleep.
- **Clips written for this arm:** the original library was made for a 6-joint robot; most of its character was
  yaw and roll, which a 3-pitch-joint lamp doesn't have (shake_no, scan_room, tail_wag shrank to 0–6°), and it
  was timed for 120 °/s servos (emote.py slowed clips 5–8x; celebrate took 24 s). The 23 `/lamp` clips use
  pitch only and are timed to the arm's measured caps.
- **Lamp principles:** the shade is the face; the head leads and the arm follows 0.15 s later; squash and
  stretch with the elbow; lean back with the shoulder, never forward (+8° forward already needs 1.39 N·m);
  "no" is a recoil and a look away, since a lamp can't shake its head.
- **Before vs after:** the old show's `lamp_curious` needed 1.91 N·m and `lamp_wake` 1.46–1.87 N·m, above the
  shoulder's lifting limit. The new show peaks at 0.72 N·m.
