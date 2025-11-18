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
