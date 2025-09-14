package org.firstinspires.ftc.teamcode.constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class Constants {

    public static final class Drive {
        public static final String FL = "fl drive";
        public static final String FR = "fr drive";
        public static final String RL = "rl drive";
        public static final String RR = "rr drive";

        public static final boolean INVERT_FL = false;
        public static final boolean INVERT_FR = true;
        public static final boolean INVERT_RL = false;
        public static final boolean INVERT_RR = true;

        public static final double WHEEL_DIAMETER_M = 0.075;

        public static final boolean FIELD_CENTRIC_DEFAULT = true;
        public static final boolean PRECISION_SCALES_ROTATION = true;
        public static final double PRECISION_SCALE = 0.35;

        public static final boolean SLEW_ENABLED = True;
        public static final double SLEW_TRANSLATION = 2.5;
        public static final double SLEW_ROTATION = 3.0;

        public static final double HEADING_LOCK_TOL_DEG = 2.0;
        public static final double HEADING_LOCK_KP_PER_DEG = 0.02;
        public static final double HEADING_LOCK_MAX_ROT_PER_SEC = 180.0;

        public static final boolean RUN_WITHOUT_ENCODERS = true;
        public static final boolean ZERO_POWER_BRAKE = true;

        public static final boolean REV_BULK_CACHE_AUTO = false;
    }

    public static final class IMUCfg {
        public static final String NAME = "imu";
        public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public static final RevHubOrientationOnRobot.UsbFacingDirection USB =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        public static final AngleUnit HEADING_UNIT = AngleUnit.RADIANS;
    }

    public static final class Pinpoint {
        public static final int I2C_PORT = 0;
        public static final int I2C_ADDRESS = 0x31;
        public static final long UPDATE_MS = 20;
        public static double TRACK_WIDTH_MM = 150.0;
        public static double FORWARD_OFFSET_MM = 0.0;
        public static final String CALIBRATION_JSON = "/sdcard/FIRST/calibration/pinpoint.json";
    }

    public static final class Vision {
        public static final String WEBCAM_NAME = "webcam1";
        public static final boolean CAMERA_INVERTED = true;
        public static final int STREAM_WIDTH = 640;
        public static final int STREAM_HEIGHT = 480;
        public static final String APRILTAG_FAMILY = "tag36h11";
        public static final double TAG_SIZE_M = 0.1651;
        public static double CAM_X = 0.0, CAM_Y = 0.0, CAM_Z = 0.0;
        public static double CAM_YAW = 0.0, CAM_PITCH = 0.0, CAM_ROLL = 0.0;
    }

    public static final class Controls {
        public static final boolean ROTATION_ON_RIGHT_STICK_X = true;

        public enum ShapingType { DEAD_EXPO, LINEAR, SQUARED }
        public static final ShapingType SHAPING = ShapingType.DEAD_EXPO;
        public static final double DEAD_BAND = 0.05;
        public static final double EXPO_TRANSLATION = 0.60;
        public static final double EXPO_ROTATION = 0.60;
    }

    public static final class TelemetryCfg {
        public static final int TELEMETRY_LEVEL = 4;
    }

    public static final class Digital {
        public static final String FLAG_A = "flag sw a";
        public static final String FLAG_B = "flag sw b";
    }

    private Constants() {}
}
