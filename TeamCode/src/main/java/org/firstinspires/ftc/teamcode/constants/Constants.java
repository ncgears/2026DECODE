package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Centralized configuration for hardware names, tuning values, and feature toggles.
 * Every constant has a short comment describing its purpose.
 */
public final class Constants {
    /** Drivetrain configuration and TeleOp behavior. */
    public static final class Drive {
        // RC config names for the four mecanum drive motors
        public static final String FL = "fl drive";
        public static final String FR = "fr drive";
        public static final String RL = "rl drive";
        public static final String RR = "rr drive";

        // Whether to reverse each motor in code to match wiring/gearbox orientation
        public static final boolean INVERT_FL = true;
        public static final boolean INVERT_FR = true;   // per user: fr inverted
        public static final boolean INVERT_RL = true;
        public static final boolean INVERT_RR = true;   // per user: rr inverted

        // Mechanism notes (used for feedforward/auton planning later)
        public static final double WHEEL_DIAMETER_M = 0.075; // 75 mm REV mecanum
        public static final double GEAR_RATIO_UP = 15.0;     // Ultra Planetary 15:1

        // Field-centric driving default and precision scaling
        public static final boolean FIELD_CENTRIC_DEFAULT = true;    // start in field-centric
        public static final boolean PRECISION_SCALES_ROTATION = true;// precision also scales rotation
        public static final double  PRECISION_SCALE = 0.35;          // g1.LB hold

        // Rotation control: use right stick X, and define CW sign (RS right => CW on field)
        public static final boolean ROTATION_RIGHT_STICK_X = true;
        public static final boolean ROTATION_CW_IS_POSITIVE = false;

        // Slew limiters to smooth commands (units per second)
        public static final boolean SLEW_ENABLED = true;
        public static final double  SLEW_TRANSLATION = 2.5;
        public static final double  SLEW_ROTATION    = 3.0;

        // Heading lock controller (used when g1.RB held or during one-shot snap)
        public static final double HEADING_LOCK_TOL_DEG = 2.0;      // acceptable error
        public static final double HEADING_LOCK_KP_PER_DEG = 0.02;  // proportional gain
        public static final double HEADING_LOCK_MAX_ROT_PER_SEC = 180.0; // clamp for turn rate
        public static final double SNAP_TOL_DEG = 3.0;              // on-target tolerance for one-shot snap

        // Motor run mode and braking preference
        public static final boolean RUN_WITHOUT_ENCODERS = true;     // TeleOp
        public static final boolean ZERO_POWER_BRAKE = true;         // brake on zero power

        // Lynx bulk caching – OFF for now (enable for performance once stable)
        public static final boolean REV_BULK_CACHE_AUTO = false;
    }

    /** IMU (BNO055 in REV hub) orientation and output units. */
    public static final class IMUCfg {
        public static final String NAME = "imu";  // RC config name
        // Physical mounting as installed (logo up, USB forward)
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB =
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        public static final AngleUnit HEADING_UNIT = AngleUnit.RADIANS;
    }

    /** Intake roller configuration. */
    public static final class Intake {
        public static final String MOTOR = "intake";  // RC config name
        public static final double POWER_IN  = 0.90;  // forward intake power
        public static final double POWER_OUT = 0.75;  // reverse outtake power
        public static final int    PAUSE_DURING_INDEX_MS = 200; // pause intake while indexer steps
    }

    /** Indexer carousel configuration (CR servo + slot/colour sensing). */
    public static final class Indexer {
        public static final String SERVO = "indexer";       // CR servo name
        public static final String SLOT_LIMIT = "indexslot";// NC slot switch (HIGH normal, LOW pressed)
        public static final String PURPLE_DI = "indexpurple"; // ACTIVE-LOW using level shifter when purple detected at S1L
        public static final String GREEN_DI  = "indexgreen";  // ACTIVE-LOW using level shifter when green detected at S1L

        public static final double POWER_FWD = 0.80;           // CR servo forward power
        public static final boolean DIR_FORWARD_IS_POSITIVE = false; // flip if wiring requires
        public static final int DEBOUNCE_MS = 25;               // debounce for slot switch
        public static final int STEP_TIMEOUT_MS = 700;          // stop & flag jam if exceeded
        public static final boolean AUTO_ADVANCE_ON_DETECT = true; // auto-step when color seen at S1L

        public static final int QUEUE_SIZE = 3; // S0 (shoot), S1L (load), S2
    }

    /** Shooter (dual flywheel) and ramp servo. */
    public static final class Shooter {
        public static final String MOTOR_1 = "shooter1"; // left/right flywheels; electrically inverted pair
        public static final String MOTOR_2 = "shooter2";
        public static final boolean MOTOR_1_INVERT = false; // flip if wiring requires
        public static final boolean MOTOR_2_INVERT = !MOTOR_1_INVERT; // flip if wiring requires
        public static final String RAMP_SERVO = "ramp";  // feed ramp servo

        public static final double IDLE_POWER   = 0.15; // always-on idle to reduce inrush
        public static final double TARGET_POWER = 0.90; // open-loop target for shots

        public static final double RAMP_RETRACTED = 0.7; // servo position for safe idle
        public static final double RAMP_ENGAGED   = 0.5; // servo position to feed S0 into shooter

        public static final int HOLD_AFTER_RELEASE_MS = 2000; // keep flywheels up to speed after RT release
        public static final int INTER_SHOT_DWELL_MS   = 300;  // dwell between auto-advances (ms)
        public static final int MAX_BURST_FEEDS       = 3;    // cap feeds per RT hold

        public static final int SPINUP_WAIT_MS   = 250; // wait at least this long before first feed
        public static final int RAMP_UP_TIME_MS  = 0;   // optional power ramp time (0 = instant)
        public static final boolean USE_RPM_MODE = false; // future switch to PIDF RPM mode
        public static final int SHOOTER_ENCODER_CPR = 28; // shooter1 built-in encoder CPR

        public static final boolean JAM_RETRACT_IMMEDIATE = true; // on jam, retract ramp & idle immediately
    }

    /** Elevator (endgame lift) configuration. */
    public static final class Elevator {
        public static final String MOTOR    = "elev";     // HD Hex motor name
        public static final String LIMIT_SW = "elevlim";  // NC HIGH normal, LOW when triggered

        public static final double POWER_UP   = 0.80;   // deploy
        public static final double POWER_DOWN = -0.40;  // retract

        public static final int RAMP_UP_MS   = 250; // soft-start ramp time up
        public static final int RAMP_DOWN_MS = 150; // soft-start ramp time down

        public static final int LOCKOUT_ENCODER_THRESHOLD = 14; // counts; above this => lockout other mechanisms
    }

    /** Vision configuration for two webcams and fusion gates. */
    public static final class Vision {
        public static final String MOTIF_CAM_NAME = "motifcam"; // reads motif tags (21,22,23)
        public static final String ODO_CAM_NAME   = "odocam";   // reads goal tags (20 blue, 24 red)
        public static final int STREAM_WIDTH  = 1280; // default stream resolution (speed)
        public static final int STREAM_HEIGHT = 720;

        // Extrinsics for odocam (measure during bring-up; meters & degrees)
        /*
            +X=forward, +y=left, +z=up; origin is center of robot rotation (X from contact point of each mecanum) at the floor
            yaw is rotation relative to the robot, pitch is tilt from horizontal, roll is the rotation relative to the camera lens
         */
        public static double ODO_X = 0.0, ODO_Y = 0.0, ODO_Z = 0.3486;
        public static double ODO_YAW = 0.0, ODO_PITCH = 0.0, ODO_ROLL = 0.0;

        // Goal tag fusion cadence and gates (used for pose correction preference)
        public static final double ODO_FUSE_RATE_HZ = 2.0; // how often to fuse when a tag is visible
        public static final double ODO_FUSE_ALPHA   = 0.20; // blend factor (placeholder for Pinpoint integration)
        public static final double ODO_MAX_RANGE_M      = 3.0;  // only fuse within this range
        public static final double ODO_MAX_ABS_YAW_DEG  = 25.0; // only fuse if facing within this yaw
        public static final double ODO_MAX_DELTA_M      = 0.1; // clamp per update
        public static final double ODO_MAX_DELTA_DEG    = 5.0;  // clamp per update
        public static final double ODO_STICK_THRESH     = 0.30; // only fuse when driver stick is calm

        // AprilTag IDs for this season
        public static final int TAG_MOTIF_GPP = 21; //21
        public static final int TAG_MOTIF_PGP = 22; //22
        public static final int TAG_MOTIF_PPG = 23; //23
        public static final int TAG_GOAL_BLUE = 20; //20
        public static final int TAG_GOAL_RED  = 24; //24

        // OdoCam lens intrinsics (for AprilTagProcessor)
        // Units are pixels for focal lengths and principal point.
        public static final double ODO_FX = 905.168;
        public static final double ODO_FY = 905.168;
        public static final double ODO_CX = 667.265;
        public static final double ODO_CY = 356.696;
    }

    /** Pinpoint calibration store (JSON) and default values until tuned. */
    public static final class Pinpoint {
        public static final String NAME = "pinpoint";
        public static String CALIBRATION_JSON = "/sdcard/FIRST/calibration/pinpoint.json"; // auto-persist path
        /*
            Geometry notes (robot frame: +X = forward, +Y = left, origin = rotation center):

            +X=forward, +y=left; origin is center of robot rotation (X from contact point of each mecanum)
            Y pod (strafe) is at x+20.6, y-92.7
            X pod (forward) is at x+20.6, y+87.2
            Track width is 179.9 (92.7+87.2)

            For the Pinpoint driver, we assume symmetric pods based on the tuned values:
                xOffset (sideways position of X pod)   = +TRACK_WIDTH_MM / 2
                yOffset (forward position of Y pod)    = FORWARD_OFFSET_MM
         */
        // Forward/back positions of pods (you mostly care about Y pod here)
        public static double X_POD_FORWARD_OFFSET_MM = 20.6;  // X pod x
        public static double Y_POD_FORWARD_OFFSET_MM = 20.6;  // Y pod x

        // Sideways positions of pods (you mostly care abouy X pod here)
        public static double X_POD_SIDE_OFFSET_MM = 87.2;   // X pod y (left is +)
        public static double Y_POD_SIDE_OFFSET_MM = -92.7;  // Y pod y (right is -)

        // Optional derived helper, if you still want to keep “track width” around:
        public static double TRACK_WIDTH_MM = Math.abs(X_POD_SIDE_OFFSET_MM - Y_POD_SIDE_OFFSET_MM);
    }

    /** Stick shaping selection and parameters. */
    public static final class Controls {
        public enum ShapingType { DEAD_EXPO, LINEAR, SQUARED }
        public static final ShapingType SHAPING = ShapingType.DEAD_EXPO; // default shaping
        public static final double DEAD_BAND = 0.05;     // 5% deadband
        public static final double EXPO_TRANSLATION = 0.60; // translation expo factor (0..1)
        public static final double EXPO_ROTATION    = 0.60; // rotation expo factor (0..1)
    }

    /** Telemetry verbosity (1..4). */
    public static final class TelemetryCfg {
        public static final int TELEMETRY_LEVEL = 4; // default verbose for bring-up
    }

    /** Digital channel names for team alliance flags. */
    public static final class Digital {
        public static final boolean SWAP_FLAG_SW = true;
        public static final String FLAG_A = "flag sw a";
        public static final String FLAG_B = "flag sw b";
        public static final String AUTO_A  = "auto sw a";
        public static final String AUTO_B  = "auto sw b";
    }

    private Constants() {}
}
