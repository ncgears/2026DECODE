# TeamCode – FTC 2026 DECODE (Java, VisionPortal)

This TeamCode drop contains:
- `TeleOp_Main` – full TeleOp implementing the agreed controls and behaviors.
- Subsystems: Drive, Intake, Indexer (CR servo), Shooter (dual flywheel + ramp), Elevator (lockout).
- Vision manager: two cameras (`motifcam` and `odocam`) via VisionPortal and AprilTag library.
- `Calibration_Pinpoint` opmode for JSON persistence of calibration values.
- `Constants` class with comments for each configurable item.

## Import
Unzip this **TeamCode** folder into your Android Studio project at:
`<project-root>/TeamCode/` (replace existing TeamCode module or merge sources).

Build with JDK **17**, module language level **11** (SDK 11+ default).

## Controls (summary)
- **Gamepad1**: LB precision, RB hold-lock, Y field/robot, X motif re-scan, BACK zero, START IMU↔Pinpoint, D-pad ←/→ cardinal snap (one-shot), A (hold) arm elevator, D-pad ↑/↓ elevator, B global panic stop.
- **Gamepad2**: X intake, LB+X outtake, LT indexer advance, START Re-check Slots, RT fire (burst≤3), A toggle previews, B mech panic stop.

See constants in `constants/Constants.java` to adjust tuning and names.