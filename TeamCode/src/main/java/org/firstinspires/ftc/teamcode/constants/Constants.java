package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Centralized configuration for drive, sensors, vision, controls, and telemetry.
 * All items are grouped so you can find/tune them quickly.
 */
public final class Constants {

    /** Drivebase and TeleOp drive behavior. */
    public static final class Drive {
        // --- Hardware names (exact strings from the RC configuration) ---
        /** Motor name: front-left (mecanum). */
        public static final String FL = "fl drive";
        /** Motor name: front-right (mecanum). */
        public static final String FR = "fr drive";
        /** Motor name: rear-left (a.k.a. back-left) (mecanum). */
        public static final String RL = "rl drive";
        /** Motor name: rear-right (a.k.a. back-right) (mecanum). */
        public static final String RR = "rr drive";

        // --- Motor inversion (true means reverse the motor direction in code) ---
        /** Set true if FL spins backward when commanded forward. */
        public static final boolean INVERT_FL = false;
        /** FR is inverted per wiring/gearbox orientation. */
        public static final boolean INVERT_FR = true;
        /** Set true if RL spins backward when commanded forward. */
        public static final boolean INVERT_RL = false;
        /** RR is inverted per wiring/gearbox orientation. */
        public static final boolean INVERT_RR = true;

        // --- Geometry / wheels ---
        /** Mecanum wheel diameter in meters (REV 75 mm = 0.075 m). */
        public static final double WHEEL_DIAMETER_M = 0.075;

        // --- Field-centric & precision behavior ---
        /** Default to field-centric drive (true) or robot-centric (false); toggle at runtime. */
        public static final boolean FIELD_CENTRIC_DEFAULT = true;
        /** If true, precision mode scales rotation as well as translation. */
        public static final boolean PRECISION_SCALES_ROTATION = true;
        /** Precision mode gain (hold LB): multiplies all inputs by this factor. */
        public static final double PRECISION_SCALE = 0.35;

        // --- Input slew limiting (smooth acceleration) ---
        /** Master enable for slew limiters on x/y/rot inputs. */
        public static final boolean SLEW_ENABLED = true;
        /** Max change per second for translation inputs (in normalized power units / s). */
        public static final double SLEW_TRANSLATION = 2.5;
        /** Max change per second for rotation input (in normalized power units / s). */
        public static final double SLEW_ROTATION = 3.0;

        // --- Heading lock (hold RB to lock to current heading) ---
        /** Allowed error around the target heading (degrees) before we consider it “on target.” */
        public static final double HEADING_LOCK_TOL_DEG = 2.0;
        /** Proportional gain: maps heading error (deg) to a rotation command (deg/s). */
        public static final double HEADING_LOCK_KP_PER_DEG = 0.02;
        /** Clamp for the heading lock rotation command (deg/s) to keep motion smooth. */
        public static final double HEADING_LOCK_MAX_ROT_PER_SEC = 180.0;

        // --- Motor run modes & braking ---
        /** Use open-loop TeleOp (RUN_WITHOUT_ENCODER) for arcade/mecanum control. */
        public static final boolean RUN_WITHOUT_ENCODERS = true;
        /** If true, set motors to BRAKE at zero power; false sets to FLOAT (coast). */
        public static final boolean ZERO_POWER_BRAKE = true;

        // --- REV hub bulk caching (loop speed optimization) ---
        /**
         * If true, enables LynxModule BulkCachingMode.AUTO (reads are cached per loop).
         * Pros: fewer I2C/serial transactions → faster loops. Cons: mid-loop sensor changes
         * aren’t visible until next cycle. Leave false unless you need the extra speed.
         */
        public static final boolean REV_BULK_CACHE_AUTO = false;
    }

    /** IMU mounting/orientation and heading readout. */
    public static final class IMUCfg {
        /** Hardware name for the IMU (BNO055 on Expansion Hub). */
        public static final String NAME = "imu";
        /** Physical mounting: which way the REV Hub logo faces. */
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        /** Physical mounting: which way the USB ports face. */
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        /** Preferred heading unit for any conversions (internal code uses radians). */
        public static final AngleUnit HEADING_UNIT = AngleUnit.RADIANS;
    }

    /** goBILDA Pinpoint odometry configuration and calibration persistence. */
    public static final class Pinpoint {
        /** REV Hub I²C bus index where Pinpoint is connected (0 or 1). */
        public static final int I2C_PORT = 0;
        /** 7-bit I²C address for Pinpoint (default 0x31). */
        public static final int I2C_ADDRESS = 0x31;
        /** TeleOp polling interval for Pinpoint (milliseconds). */
        public static final long UPDATE_MS = 20;
        /** Lateral distance between left/right pods (mm). Tuned via calibration OpMode. */
        public static double TRACK_WIDTH_MM = 150.0;   // placeholder until tuned
        /** Forward (+) or backward (−) offset of the tracking center from robot center (mm). */
        public static double FORWARD_OFFSET_MM = 0.0;  // placeholder until tuned
        /** Path where calibration JSON is saved/loaded on the RC phone. */
        public static final String CALIBRATION_JSON = "/sdcard/FIRST/calibration/pinpoint.json";
    }

    /** Vision (USB webcam + AprilTags via VisionPortal). */
    public static final class Vision {
        /** Webcam config name in the Robot Controller hardware map. */
        public static final String WEBCAM_NAME = "webcam1";
        /** True if the physical camera is mounted upside-down (inverted). */
        public static final boolean CAMERA_INVERTED = true;
        /** Video stream width (pixels) requested from VisionPortal. */
        public static final int STREAM_WIDTH = 640;
        /** Video stream height (pixels) requested from VisionPortal. */
        public static final int STREAM_HEIGHT = 480;
        /** AprilTag family string (FTC DECODE uses tag36h11). */
        public static final String APRILTAG_FAMILY = "tag36h11";
        /** Physical tag size (edge length) in meters (6.5 in = 0.1651 m). */
        public static final double TAG_SIZE_M = 0.1651;

        // Camera pose relative to the robot center; measure during bring-up for accurate tag poses.
        /** Camera X offset in meters (+ forward). */
        public static double CAM_X = 0.0;
        /** Camera Y offset in meters (+ left). */
        public static double CAM_Y = 0.0;
        /** Camera Z offset in meters (+ up). */
        public static double CAM_Z = 0.0;
        /** Camera yaw in degrees (CCW+, around Z). */
        public static double CAM_YAW = 0.0;
        /** Camera pitch in degrees (nose up +, around Y). */
        public static double CAM_PITCH = 0.0;
        /** Camera roll in degrees (CCW+, around X). */
        public static double CAM_ROLL = 0.0;
    }

    /** Control mappings and input shaping. */
    public static final class Controls {
        /** If true, rotation is read from gamepad1 right-stick X; otherwise left-stick X. */
        public static final boolean ROTATION_ON_RIGHT_STICK_X = true;

        /** Which input shaping style to use for sticks. */
        public enum ShapingType { DEAD_EXPO, LINEAR, SQUARED }
        /** Default shaping: deadband + cubic expo blend for finer low-speed control. */
        public static final ShapingType SHAPING = ShapingType.DEAD_EXPO;
        /** Deadband applied to sticks (0.05 = 5%). */
        public static final double DEAD_BAND = 0.05;
        /** Expo mix for translation sticks (0 = linear, 1 = fully cubic). */
        public static final double EXPO_TRANSLATION = 0.60;
        /** Expo mix for rotation stick (0 = linear, 1 = fully cubic). */
        public static final double EXPO_ROTATION = 0.60;
    }

    /** Telemetry output level (1..4 where 4 is most verbose). */
    public static final class TelemetryCfg {
        /** Global telemetry verbosity: lower = quieter; higher = more detail. */
        public static final int TELEMETRY_LEVEL = 4;
    }

    /** Digital inputs used to determine alliance color via installed flags. */
    public static final class Digital {
        /** Digital channel name for “flag switch A” (pull-up; high when open). */
        public static final String FLAG_A = "flag sw a";
        /** Digital channel name for “flag switch B” (pull-up; high when open). */
        public static final String FLAG_B = "flag sw b";
    }

    /** Prevent instantiation. */
    private Constants() {}
}
