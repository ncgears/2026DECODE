## Driver Controls (g1)

### Drive

- **Translation**
    - **Left stick (LS)**: robot translation
        - LS X → strafe left/right
        - LS Y → forward/back
    - Input is passed through a shaping function (non-linear curve for finer low-speed control).

- **Rotation**
    - **Right stick X (RS.X)**: rotation
        - Positive RS.X = counter-clockwise (CCW) rotation at the drive layer
        - Internally, sign may be flipped based on `ROTATION_CW_IS_POSITIVE`, but from the driver’s perspective:
            - Push stick right → rotate right
            - Push stick left → rotate left

- **Field-centric vs Robot-centric**
    - Drive math runs in either **field-centric** or **robot-centric** mode.
    - In field-centric:
        - LS is interpreted in field coordinates (up on stick = “downfield”, regardless of robot heading).
    - In robot-centric:
        - LS is interpreted in robot coordinates (up on stick = “forward relative to robot”).

### Drive Mode & Heading Management

- **`BACK`**
    - On press edge (not hold): **zero the current heading** at the heading provider
        - Resets “forward” reference for field-centric drive.

- **`START`**
    - On press edge:
        - If **LB is NOT held**:
            - Toggle **drive frame**:
                - Field-centric ↔ Robot-centric
        - If **LB IS held**:
            - Toggle **heading provider**:
                - IMU heading ↔ Pinpoint heading
            - Used to pick which sensor defines the current heading for field-centric math.

### Precision / Slow Mode

- **Left bumper (LB)**
    - While held:
        - Scales **translation** (LS X/Y) by a configured precision factor.
        - Optionally also scales rotation (RS.X) if enabled via config.
    - Effect: more precise low-speed driving for lining up shots / intakes.

### Elevator (Driver-gated)

- **A button (A)**
    - Acts as an **“arm”/enable** for driver elevator control.
    - While **A is held**:
        - **D-pad Up**: command elevator **up**
        - **D-pad Down**: command elevator **down**
    - When **A is released**:
        - Elevator receives no up/down commands from the driver (driver cannot move it by accident).

### Odometry / Vision Fusion Gating (implicit)

- There is no explicit button for this, but it’s worth noting:
    - OdoCam fusion during TeleOp is **enabled only when**:
        - The magnitude of **g1 left stick** is below a configured threshold (robot mostly not driving), and
        - The elevator is not in a lockout state, and
        - `ODO_FUSE_DURING_TELEOP` is enabled in constants.
    - From the driver’s perspective: **drive gently or stop** to allow odometry/vision fusion to kick in cleanly.


## Operator – Gamepad 2 (Logitech F310)

### Intake

- **X**
    - Intake forward (pull artifacts in).

- **LB + X**
    - Outtake / reverse intake (push artifacts out).


### Indexer

- **LT (tap)**
    - When not reindexing, not unjamming, and the indexer is idle:
        - Starts a single forward step (`indexer.startStep()`), advancing the carousel one slot.

- **D-pad LEFT (hold)** – *Unjam*
    - Press: `indexer.startUnjamReverse()` – run the indexer backward to clear a jam.
    - Release: `indexer.stopUnjam()`, and if the indexer is idle, `indexer.startStep()` is issued to re-acquire a slot.

- **B (tap)** – *Clear + rescan + motif-rotate*  
  When the indexer is idle and not unjamming:
    1. Clears the logical queue (`indexer.clearAll()`).
    2. Enters a reindex/rescan sequence to refill S0/S1L/S2.
    3. Once S0/S1L/S2 are all non-`NONE` and the step completes, calls
       `indexer.rotateForMotif(matchMotifCode)` if the motif code is valid.

- **LB + B (tap)** – *Cycle static motif*
    - Cycles the TeleOp static motif code:
        - `NONE/other → PGP → GPP → PPG → PGP → ...`
    - Does **not** move the indexer; just changes `matchMotifCode` used for rotations.


### Shooter

- **RT (hold)** – *Shot sequence*
    - Uses `ShooterSubsystem.handleRightTrigger()` for spin-up, ramp control,
      indexer shot steps, and streaming fire.

- **Y (tap)** – *Shooter stop*
    - Immediately sets shooter target power to `0.0` and resets the fire control state
      machine (`stop()` + `resetFireControl()`).

- **TeleOp start behavior**
    - On `start()`, shooter is commanded to **idle** (`idle()` + `resetFireControl()`).
