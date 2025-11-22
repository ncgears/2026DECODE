# Road Runner + Pinpoint Calibration (Remaining Steps)

**Preconditions (already completed):**

- ✅ `LocalizationTest` verified:
    - +X = forward
    - +Y = left
    - CCW heading positive
- ✅ `inPerTick` and `lateralInPerTick` tuned and behaving correctly

This guide covers the remaining calibration steps that require field space:

1. Forward ramp logging → initial `kS`, `kV`
2. Manual feedforward tuning → refine `kS`, `kV`, pick `kA`
3. Angular ramp logging → `trackWidthTicks` (and optionally Pinpoint geometry refinement)
4. Manual feedback tuning → path-following gains
5. Final validation → `LocalizationTest` + `SplineTest`

Robot code specifics (for reference):

- Feedforward constants: `teamcode/Constants.java` → `Constants.RoadRunner`
- Drive model params: `teamcode/drive/MecanumDrive.java` → `MecanumDrive.Params`
- Tuning opmodes: `tuning` package and RR-provided opmodes via `TuningOpModes`

---

## 1. ForwardRampLogger → initial kS, kV

**OpMode:** `ForwardRampLogger`  
**Goal:** Get first-pass `kS` and `kV` for straight-line motion.

### Setup

1. Fully charged battery.
2. Long straight lane on the field, robot pointing “forward”.
3. No obstacles in the robot’s path.

### Procedure

1. On Driver Station:
    - Select `ForwardRampLogger`.
    - Run the opmode.
2. Let the robot drive forward through the ramp:
    - If it’s going to hit something, stop the opmode early.
3. On a laptop:
    - Connect to FTC Dashboard (same Wi‑Fi as RC).
    - Open the Forward Ramp tuning page (URL will be shown in telemetry / docs).
    - Load the latest run.
    - Use the built-in line fit:
        - Line **intercept** → `kS`
        - Line **slope** → `kV`

### Edit

Update `teamcode/Constants.java`:

```java
public static final class RoadRunner {
    public static final double MAX_VEL = 30.0;
    public static final double MAX_ACCEL = 30.0;
    public static final double MAX_ANG_VEL = Math.PI;
    public static final double MAX_ANG_ACCEL = Math.PI;

    public static double kS = /* intercept from ForwardRampLogger */;
    public static double kV = /* slope from ForwardRampLogger */;
    public static double kA = 0.0;  // will tune next
}
```

Rebuild + deploy.

**Done when:** The line fit looks sane and you have non-zero, non-ridiculous `kS` and `kV` values.

---

## 2. ManualFeedforwardTuner → refine kS, kV, choose kA

**OpMode:** `ManualFeedforwardTuner`  
**Goal:** Make measured velocity `v` track reference velocity `vRef` using `kS`, `kV`, `kA`.

### Setup

1. Same straight run as ForwardRampLogger.
2. Dashboard connected and graphing.

### Procedure

1. On Driver Station:
    - Run `ManualFeedforwardTuner`.
2. On Dashboard:
    - Graph:
        - `vRef` (reference velocity)
        - `v` (measured velocity)
3. In code (`Constants.RoadRunner`), iterate:

    1. Start with `kS`, `kV` from ForwardRampLogger.
    2. Set a small non-zero `kA`, for example:

       ```java
       public static double kA = 1e-7;
       ```

    3. Rebuild + deploy.
    4. Run `ManualFeedforwardTuner` again and watch `v` vs `vRef`.

### Tuning heuristics

- `v` consistently low compared to `vRef`:
    - Increase `kV` slightly.
- `v` lags on acceleration (slow to “catch up” when speed ramps up):
    - Increase `kA` in small steps.
- `v` overshoots / oscillates around `vRef`:
    - Decrease `kA` a bit and/or slightly reduce `kV`.
- Robot “jumps” from a stop:
    - `kS` too high; lower it.
- Robot struggles to start moving:
    - `kS` too low; raise it slightly.

Repeat until `v` tracks `vRef` reasonably well and the robot moves smoothly.

### Edit

Write final values into `Constants.RoadRunner`:

```java
public static double kS = /* final tuned value */;
public static double kV = /* final tuned value */;
public static double kA = /* final tuned value */;
```

Rebuild + deploy before moving on.

---

## 3. AngularRampLogger → trackWidthTicks (+ optional geometry refinement)

**OpMode:** `AngularRampLogger`  
**Goal:** Calibrate `trackWidthTicks` for the mecanum model and optionally refine the effective Pinpoint geometry.

`trackWidthTicks` lives in `MecanumDrive.Params`:

```java
public static class Params {
    public double inPerTick = ...;          // already tuned
    public double lateralInPerTick = ...;   // already tuned
    public double trackWidthTicks = 0.0;    // to be tuned
    // ...
}
```

### Setup

1. Robot centered on a tile with room to spin in place.
2. Good traction (no sliding).
3. Enough cable slack / battery secured, etc.

### Procedure

1. On Driver Station:
    - Run `AngularRampLogger`.
2. Robot will spin in place with ramping angular velocity.
3. On Dashboard:
    - Open the Angular Ramp tuning page.
    - Load the latest run.
    - Use the “dead-wheel / localizer-aware” analysis (the one tied to the Pinpoint encoder group if available).
4. Note the recommended value for:
    - `trackWidthTicks`
    - Any suggested “effective track width” / pod positions (if the page reports them).

### Edit

Update `MecanumDrive.Params`:

```java
public static class Params {
    public double inPerTick = ...;
    public double lateralInPerTick = ...;

    public double trackWidthTicks = /* from AngularRampLogger */;

    public double axialGain = 2.0;
    public double lateralGain = 2.0;
    public double headingGain = 4.0;

    public double axialVelGain = 0.0;
    public double lateralVelGain = 0.0;
    public double headingVelGain = 0.0;

    public double kS = Constants.RoadRunner.kS;
    public double kV = Constants.RoadRunner.kV;
    public double kA = Constants.RoadRunner.kA;
}
```

Optionally, if the analysis reports a significantly different effective track width than what `Constants.Pinpoint.TRACK_WIDTH_MM` gives you, you can slightly adjust the pod offsets in `Constants.Pinpoint` to match, but don’t overfit small noise.

**Done when:** In-place spins look clean (no large translation), and the logger-derived `trackWidthTicks` is set.

---

## 4. ManualFeedbackTuner → path-following gains

**OpMode:** `tuning.ManualFeedbackTuner`  
**Goal:** Tune the position and heading gains of the path-following controller.

`ManualFeedbackTuner` runs a simple back-and-forth path:

```java
Pose2d(0, 0, 0)
→ lineToX(DISTANCE)
→ lineToX(0)
```

Gains live in `MecanumDrive.Params`:

```java
public double axialGain = 2.0;
public double lateralGain = 2.0;
public double headingGain = 4.0;

public double axialVelGain = 0.0;
public double lateralVelGain = 0.0;
public double headingVelGain = 0.0;
```

### Setup

1. Robot on a straight field lane with at least `DISTANCE` inches of space.
2. Optionally connect Dashboard to view pose/error.

### Procedure

1. Run `ManualFeedbackTuner`.
2. Let the robot run several cycles:
    - Out to `DISTANCE`, back to 0.
3. Observe:
    - Overshoot at ends?
    - Oscillation?
    - Sideways drift?
    - Poor heading control?

### Tuning heuristics

- **Axial (forward/back) error:**
    - Overshoot / oscillation at endpoints:
        - Decrease `axialGain`.
    - Sluggish convergence:
        - Increase `axialGain`.
- **Heading control:**
    - Robot arcs or doesn’t correct heading:
        - Increase `headingGain`.
    - Robot over-corrects / oscillates in heading:
        - Decrease `headingGain`.
- **Lateral (sideways) error:**
    - Robot drifts sideways and does not recover:
        - Increase `lateralGain`.
    - Robot wobbles side-to-side:
        - Decrease `lateralGain`.

Leave the velocity gains at 0 unless you deliberately want more advanced control.

### Edit

Update `MecanumDrive.Params` with your final gains and rebuild + deploy.

**Done when:** Robot goes out and back along the line cleanly, with minimal overshoot and drift.

---

## 5. Final Validation

### 5.1 LocalizationTest (post-tune check)

**OpMode:** `tuning.LocalizationTest`  
**Goal:** Confirm overall localization quality after all tuning.

Procedure:

1. Drive the robot around the field:
    - Straight lines.
    - Large arcs.
    - Boxes / figure-eights.
2. Watch the Dashboard field overlay:
    - The drawn robot should roughly follow the real robot.
    - Small drift over long runs is acceptable; large scale errors are not.

If you see consistent scale error (e.g., always 10% short):

- Re-check:
    - `inPerTick`
    - `lateralInPerTick`
    - (Optionally) `Constants.Pinpoint.X_MULT` / `Y_MULT` if you decide to use them.

Fix systematic scale issues there, not with feedforward/feedback gains.

### 5.2 SplineTest (end-to-end path test)

**OpMode:** `tuning.SplineTest`  
**Goal:** Ensure path-following with the tuned model behaves as expected.

Procedure:

1. Set the robot at the assumed start pose used by `SplineTest` (usually `(0, 0, 0)` aligned to field axes).
2. Run `SplineTest`.
3. Observe:
    - Trajectory shape matches what Dashboard shows.
    - End pose (position + heading) is reasonably close to expected.
    - No backwards-when-it-should-be-forwards or other sign errors.

**Done when:**

- `LocalizationTest` shows sane pose tracking.
- `SplineTest` runs cleanly and lands near the expected end pose.
- Robot motion is smooth and controllable along paths.

At this point, Road Runner + Pinpoint are calibrated well enough for real autonomous trajectories, and higher-level work (adding game-specific paths and vision fusion) can sit on top of this baseline.
