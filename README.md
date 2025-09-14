# FTC 2026 DECODE — TeamCode Drop-in (Java, SDK 11+)

This ZIP contains **only the TeamCode Java sources**. Import steps:

1. Clone or open the official **FTC Robot Controller** SDK for the 2026 season (SDK 11+).
2. In Android Studio, locate the `TeamCode/src/main/java/` folder.
3. Copy the contents of this ZIP's `TeamCode/src/main/java/` into your project's `TeamCode/src/main/java/` (merge folders).
4. Sync Gradle and build. No extra dependencies are required.

## What’s included
- `TeleOp_Drive` — Field-centric mecanum, precision mode (LB), hold-to-lock heading (RB), Y to toggle field/robot, BACK to re-zero, START toggles heading source (IMU↔Pinpoint).
- `Calibration_Pinpoint` — Tune **track width** and **forward offset**; persists to `/sdcard/FIRST/calibration/pinpoint.json`.
- `Vision_AprilTags_Test` — VisionPortal + AprilTagProcessor (36h11) at 640×480.
- `Constants` — One place for all tunables and device names.
- `AllianceDetector` — Reads two digital inputs: `flag sw a`, `flag sw b`.

## Hardware mapping used
- Motors: `fl drive`, `fr drive`, `rl drive`, `rr drive` (FR & RR inverted).
- IMU: `imu` (BNO055 on Expansion Hub), mounted **logo UP**, **USB FORWARD**.
- Webcam: `webcam1` (Logitech C310), mounted **inverted**.
- Pinpoint: I²C port 0 @ 0x31 (not used by code yet; placeholder).
- Digital: `flag sw a`, `flag sw b`.

## Notes
- The `PinpointHeading` is a **stub** to keep the project compiling without the vendor driver. Replace it with the official goBILDA Pinpoint API when ready.
- AprilTag library metadata is not set; detection works without it. Tag size is set to **0.1651 m (6.5 in)** in `Constants`.
- REV Bulk Caching is **disabled by default** (toggle in `Constants`).

## Controls (Logitech F310)
- **Driver (gamepad1)**: left stick = translate (x/y), right stick X = rotate.
  - **LB** = precision (0.35), scales rotation too.
  - **RB** = hold-to-lock heading (locks to current heading).
  - **Y** = toggle field/robot centric.
  - **BACK** = re-zero heading.
  - **START** = toggle heading source (IMU ↔ Pinpoint).
- **Operator (gamepad2)**: reserved for mechanisms (future).

## Next steps
- Measure and set the camera transform in `Constants.Vision` (x,y,z,yaw,pitch,roll).
- Run `Calibration_Pinpoint` to determine odometry track width and forward offset.
- Integrate the official goBILDA Pinpoint driver and swap `PinpointHeading` implementation.
- Add mechanism subsystems and TeleOp controls (gamepad2).