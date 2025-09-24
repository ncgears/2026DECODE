package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.*;

@TeleOp(name = "TeleOp_Drive", group = "Main")
public class TeleOp_Drive extends OpMode {
    private DriveSubsystem drive;
    private HeadingProvider imuHeading, pinpointHeading, heading;
    private boolean useField = Constants.Drive.FIELD_CENTRIC_DEFAULT;

    private boolean precisionHeld = false;
    private boolean headingLockHeld = false;
    private double lockHeadingRad = 0.0;

    private boolean prevY = false, prevStart = false;

    private SlewRateLimiter slewX, slewY, slewR;
    private TelemetryUtil T;

    @Override public void init() {
        T = new TelemetryUtil(this);
        drive = new DriveSubsystem(hardwareMap);
        imuHeading = new IMUHeading(hardwareMap);
        pinpointHeading = new PinpointHeading(hardwareMap);
        heading = imuHeading;

        slewX = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewY = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewR = new SlewRateLimiter(Constants.Drive.SLEW_ROTATION);
    }

    @Override public void init_loop() {
        imuHeading.zeroHeading(); // auto-zero before start
        T.t(2, "Heading(src)", "IMU");
    }

    @Override public void loop() {
        Gamepad g1 = gamepad1;

        // Rising-edge toggles
        if (g1.y && !prevY) useField = !useField;
        if (g1.start && !prevStart) heading = (heading == imuHeading) ? pinpointHeading : imuHeading;
        prevY = g1.y; prevStart = g1.start;

        if (g1.back) { heading.zeroHeading(); }

        // Precision & lock (hold)
        precisionHeld = g1.left_bumper;
        boolean rb = g1.right_bumper;
        if (rb && !headingLockHeld) {
            headingLockHeld = true;
            lockHeadingRad = heading.getHeadingRad();
        } else if (!rb && headingLockHeld) {
            headingLockHeld = false;
        }

        // Read sticks
        double lx = g1.left_stick_x;   // + right
        double ly = -g1.left_stick_y;  // up = forward
        double rx = Constants.Drive.ROTATION_RIGHT_STICK_X ? g1.right_stick_x : g1.left_stick_x;

        // Shape
        lx = Shaping.shapeTranslate(lx);
        ly = Shaping.shapeTranslate(ly);
        rx = Shaping.shapeRotate(rx);

        // Precision scale
        double scale = precisionHeld ? Constants.Drive.PRECISION_SCALE : 1.0;
        lx *= scale; ly *= scale; if (Constants.Drive.PRECISION_SCALES_ROTATION) rx *= scale;

        // Field-centric
        double x = lx, y = ly;
        if (useField) {
            double h = heading.getHeadingRad();
            double cos = Math.cos(-h), sin = Math.sin(-h);
            double tx = x * cos - y * sin;
            double ty = x * sin + y * cos;
            x = tx; y = ty;
        }

        // Heading lock
        double rot;
        if (headingLockHeld) {
            double errDeg = Math.toDegrees(wrap(lockHeadingRad - heading.getHeadingRad()));
            double cmdDegPerSec = clip(errDeg * Constants.Drive.HEADING_LOCK_KP_PER_DEG,
                    -Constants.Drive.HEADING_LOCK_MAX_ROT_PER_SEC,
                    Constants.Drive.HEADING_LOCK_MAX_ROT_PER_SEC);
            rot = cmdDegPerSec / 360.0;
        } else {
            rot = rx;
        }

        // Slew (optional)
        if (Constants.Drive.SLEW_ENABLED) {
            x = slewX.calculate(x);
            y = slewY.calculate(y);
            rot = slewR.calculate(rot);
        }

        drive.driveRobotCentric(x, y, rot);

        // Telemetry
        T.t(4, "FieldCentric", useField);
        T.t(4, "Precision", precisionHeld);
        T.t(3, "HeadingSrc", (heading == imuHeading) ? "IMU" : "Pinpoint");
        T.t(3, "Heading(rad)", heading.getHeadingRad());
        if (headingLockHeld) T.t(3, "Lock(rad)", lockHeadingRad);
        T.t(2, "x,y,rot", String.format("%.2f, %.2f, %.2f", x, y, rot));
        telemetry.update();
    }

    private static double wrap(double a) {
        while (a > Math.PI) a -= 2*Math.PI; while (a < -Math.PI) a += 2*Math.PI; return a;
    }
    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
