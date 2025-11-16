package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.vision.AprilTagVisionManager;

import org.json.JSONObject;
import java.io.File;
import java.io.FileReader;

@TeleOp(name = "TeleOp_Main", group = "Main")
@Disabled
public class TeleOp_Main extends OpMode {
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevator;
    private HeadingProvider imuHeading, pinpointHeading, heading;
    private AprilTagVisionManager vision;
    private AllianceDetector allianceDetector;
    private TelemetryUtil T;

    private boolean useField = Constants.Drive.FIELD_CENTRIC_DEFAULT;
    private SlewRateLimiter slewX, slewY, slewR;
    private boolean rtPrev=false; private long spinStartMs=0; private long holdReleaseStartMs=0; private int burstFeeds=0;

    private boolean initHomed=false; private RecheckTask recheck=null;

    @Override public void init() {
        loadPinpointCalibrationIfPresent();
        T = new TelemetryUtil(this);
        drive = new DriveSubsystem(hardwareMap);
        imuHeading = new IMUHeading(hardwareMap);
        pinpointHeading = new PinpointHeading(hardwareMap);
        heading = imuHeading;
        intake = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        vision = new AprilTagVisionManager(hardwareMap, T);
        allianceDetector = new AllianceDetector(hardwareMap);
        slewX = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewY = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewR = new SlewRateLimiter(Constants.Drive.SLEW_ROTATION);
        indexer.startStep();
    }

    private void loadPinpointCalibrationIfPresent() {
        try {
            File f = new File(Constants.Pinpoint.CALIBRATION_JSON);
            if (!f.exists()) return;
            StringBuilder sb = new StringBuilder();
            try (FileReader fr = new FileReader(f)) {
                int c;
                while ((c = fr.read()) != -1) sb.append((char) c);
            }
            JSONObject o = new JSONObject(sb.toString());

            // Load scalar tuning values from JSON (track width & forward offset in mm)
            double trackWidth = o.optDouble(
                    "trackWidthMm",
                    Math.abs(Constants.Pinpoint.X_POD_SIDE_OFFSET_MM - Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM)
            );
            double fwdOffset = o.optDouble(
                    "forwardOffsetMm",
                    0.5 * (Constants.Pinpoint.X_POD_FORWARD_OFFSET_MM + Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM)
            );

            // 1) Forward offsets: keep both pods at the same forward distance from the CoR.
            Constants.Pinpoint.X_POD_FORWARD_OFFSET_MM = fwdOffset;
            Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM = fwdOffset;

            // 2) Side offsets: scale the existing asymmetric geometry to match trackWidth.
            double left0  = Math.abs(Constants.Pinpoint.X_POD_SIDE_OFFSET_MM);  // left pod (+Y)
            double right0 = Math.abs(Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM);  // right pod (-Y)
            double baseTrack = left0 + right0;
            if (baseTrack > 1e-6) {
                double scale = trackWidth / baseTrack;
                double newLeft  = left0  * scale;
                double newRight = right0 * scale;

                // Left pod is +Y, right pod is -Y
                Constants.Pinpoint.X_POD_SIDE_OFFSET_MM =  newLeft;
                Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM = -newRight;

                // Keep convenience TRACK_WIDTH_MM in sync
                Constants.Pinpoint.TRACK_WIDTH_MM =
                        Math.abs(Constants.Pinpoint.X_POD_SIDE_OFFSET_MM - Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM);
            }

            telemetry.addLine("Pinpoint calib loaded");
        } catch (Exception e) {
            telemetry.addLine("Pinpoint calib load failed");
        }
    }

    @Override public void init_loop() {
        indexer.loop();
        if (!initHomed && !indexer.isStepping()) {
            initHomed = true;
            recheck = new RecheckTask(indexer, shooter, intake, new TelemetryUtil(this));
        }
        if (recheck != null) {
            recheck.loop();
            if (recheck.isDone()) recheck = null;
        }
        telemetry.update();
    }

    @Override public void start() { rtPrev=false; spinStartMs=0; holdReleaseStartMs=0; burstFeeds=0; }

    @Override public void loop() {
        Gamepad g1 = gamepad1, g2 = gamepad2;
        Alliance alliance = allianceDetector.determineAlliance();

        // Elevator arming (hold g1.A)
        boolean armElevator = g1.a;
        if (armElevator) { elevator.commandUp(g1.dpad_up); elevator.commandDown(g1.dpad_down); }
        else { elevator.commandUp(false); elevator.commandDown(false); }
        elevator.loop();
        boolean lockout = elevator.isLockout();

        double stickMag = Math.hypot(g1.left_stick_x, g1.left_stick_y);
        boolean allowOdoFuse = !lockout && (stickMag < Constants.Vision.ODO_STICK_THRESH);
        boolean preferRed = (alliance == Alliance.RED);
        boolean allowMotifScan = !lockout;
        vision.loop(lockout, allowMotifScan, allowOdoFuse, preferRed);

        if (g1.b) { drive.stop(); intake.stop(); indexer.stopServo(); shooter.stop(); elevator.stop(); telemetry.update(); return; }
        if (g2.b) { intake.stop(); indexer.stopServo(); shooter.stop(); }

        if (g1.y) useField = !useField;
        if (g1.back) heading.zeroHeading();
        if (g1.start && !g1.back) { heading = (heading == imuHeading) ? pinpointHeading : imuHeading; }

        boolean intakeFwd = g2.x && !g2.left_bumper;
        boolean outtake   = g2.x &&  g2.left_bumper;
        if      (outtake)   intake.outtake();
        else if (intakeFwd) intake.intake();
        else                intake.stop();

        if (g2.left_trigger > 0.5 && !indexer.isStepping()) indexer.startStep();
        indexer.loop();

        boolean rt = g2.right_trigger > 0.5;
        if (rt && !rtPrev) { shooter.runToTarget(); spinStartMs=System.currentTimeMillis(); holdReleaseStartMs=0; burstFeeds=0; }
        if (rt) {
            shooter.runToTarget();
            long wait = Math.max(Constants.Shooter.SPINUP_WAIT_MS, Constants.Shooter.RAMP_UP_TIME_MS);
            boolean ready = (System.currentTimeMillis() - spinStartMs) >= wait;
            if (ready && !indexer.isStepping() && burstFeeds < Constants.Shooter.MAX_BURST_FEEDS) {
                shooter.setRampEngaged(true);
                indexer.startStep();
                burstFeeds++;
                spinStartMs = System.currentTimeMillis() - (Constants.Shooter.SPINUP_WAIT_MS - Constants.Shooter.INTER_SHOT_DWELL_MS);
            }
        } else {
            shooter.setRampEngaged(false);
            if (holdReleaseStartMs == 0) holdReleaseStartMs = System.currentTimeMillis();
            if (System.currentTimeMillis() - holdReleaseStartMs >= Constants.Shooter.HOLD_AFTER_RELEASE_MS) {
                shooter.idle(); burstFeeds = 0;
            }
        }
        rtPrev = rt; shooter.loop();

        double lx = g1.left_stick_x, ly = -g1.left_stick_y, rx = g1.right_stick_x;
        lx = Shaping.translate(lx); ly = Shaping.translate(ly); rx = Shaping.rotate(rx);
        double scale = g1.left_bumper ? Constants.Drive.PRECISION_SCALE : 1.0;
        lx *= scale; ly *= scale; if (Constants.Drive.PRECISION_SCALES_ROTATION) rx *= scale;

        double x = lx, y = ly;
        if (useField) {
            double h = heading.getHeadingRad();
            double cos = Math.cos(-h), sin = Math.sin(-h);
            double tx = x * cos - y * sin;
            double ty = x * sin + y * cos;
            x = tx; y = ty;
        }
        double rot = rx * (Constants.Drive.ROTATION_CW_IS_POSITIVE ? -1.0 : 1.0);
        drive.driveRobotCentric(x, y, rot);

        telemetry.update();
    }

    private static class RecheckTask {
        public RecheckTask(IndexerSubsystem a, ShooterSubsystem b, IntakeSubsystem c, TelemetryUtil T) {}
        public void loop() {}
        public boolean isDone() { return true; }
    }
}
