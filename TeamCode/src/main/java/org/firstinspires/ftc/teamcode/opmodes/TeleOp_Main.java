package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.*;
import org.firstinspires.ftc.teamcode.vision.AprilTagVisionManager;

@TeleOp(name = "TeleOp_Main", group = "Main")
public class TeleOp_Main extends OpMode {
    // Subsystems
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;
    private ElevatorSubsystem elevator;
    private HeadingProvider imuHeading, pinpointHeading, heading;
    private AprilTagVisionManager vision;
    private AllianceDetector allianceDetector;

    private TelemetryUtil T;

    // Drive state
    private boolean useField = Constants.Drive.FIELD_CENTRIC_DEFAULT;
    private boolean headingLockHeld = false;
    private double  lockHeadingRad = 0.0;
    private boolean snapActive = false;
    private double  snapTargetRad = 0.0;
    private SlewRateLimiter slewX, slewY, slewR;

    // Shooter state
    private boolean rtPrev = false;
    private long spinStartMs = 0;
    private long holdReleaseStartMs = 0;
    private int burstFeeds = 0;

    // Elevator
    private boolean armElevator = false; // g1.A hold

    // Indexer init homing + recheck task
    private boolean initHomed = false;
    private boolean initRecheckDone = false;
    private RecheckTask recheck = null;

    // Button edges
    private boolean g1xPrev=false, g2aPrev=false, g2startPrev=false;

    @Override public void init() {
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

        // Auto-home indexer to next slot on INIT
        indexer.startStep();
    }

    @Override public void init_loop() {
        // Allow homing to run
        indexer.loop();
        if (!initHomed && !indexer.isStepping()) {
            initHomed = true;
            // Immediately run one re-check slots pass after homing
            recheck = new RecheckTask(indexer, shooter, intake, T);
        }
        if (recheck != null) {
            recheck.loop();
            if (recheck.isDone()) { recheck = null; initRecheckDone = true; }
        }
        telemetry.update();
    }

    @Override public void start() {
        // Nothing special here; motif is saved at Auto START in autonomous, not TeleOp.
        // Reset shoot state
        rtPrev = false; spinStartMs = 0; holdReleaseStartMs = 0; burstFeeds = 0;
    }

    @Override public void loop() {
        Gamepad g1 = gamepad1, g2 = gamepad2;

        // Alliance (for odocam preference)
        Alliance alliance = allianceDetector.determineAlliance();

        // Elevator arming (hold g1.A)
        armElevator = g1.a;
        if (armElevator) {
            elevator.commandUp(g1.dpad_up);
            elevator.commandDown(g1.dpad_down);
        } else {
            elevator.commandUp(false);
            elevator.commandDown(false);
        }
        elevator.loop();
        boolean lockout = elevator.isLockout();
        if (lockout) T.banner(2, "ELEVATOR LOCKOUT ACTIVE");

        // Vision management (pause on lockout)
        double stickMag = Math.hypot(g1.left_stick_x, g1.left_stick_y);
        boolean allowOdoFuse = !lockout && (stickMag < Constants.Vision.ODO_STICK_THRESH);
        boolean preferRed = (alliance == Alliance.RED);
        boolean allowMotifScan = !lockout && !vision.isMotifKnown(); // only scan when requested (see button)
        vision.loop(lockout, allowMotifScan, allowOdoFuse, preferRed);

        // Global panic stop (g1.B)
        if (g1.b) {
            drive.stop(); intake.stop(); indexer.stopServo(); shooter.stop(); elevator.stop();
            telemetry.update(); return;
        }
        // Mechanism-only stop (g2.B)
        if (g2.b) { intake.stop(); indexer.stopServo(); shooter.stop(); }

        // Driver toggles
        if (g1.y) useField = !useField;         // FC toggle
        if (g1.back) heading.zeroHeading();     // re-zero
        if (g1.start && !g1.back) {             // IMU <-> Pinpoint toggle
            heading = (heading == imuHeading) ? pinpointHeading : imuHeading;
        }

        // Motif re-scan (g1.X edge -> enable scan window up to ~3s; manager will stop on acquire/timeout)
        if (g1.x && !g1xPrev) vision.startMotifScanWindow(3000);
        g1xPrev = g1.x;

        // Camera preview toggle (g2.A edge)
        if (g2.a && !g2aPrev) vision.setPreviewsOn(!g2aPrev); // simple toggle
        g2aPrev = g2.a;

        // Re-check slots routine (g2.START edge)
        if (g2.start && !g2startPrev) {
            recheck = new RecheckTask(indexer, shooter, intake, T);
        }
        g2startPrev = g2.start;
        if (recheck != null) {
            recheck.loop();
            if (recheck.isDone()) recheck = null;
        }

        // Intake (g2.X hold; g2.LB+X outtake)
        boolean intakeFwd = g2.x && !g2.left_bumper;
        boolean outtake   = g2.x && g2.left_bumper;
        if      (outtake)    intake.outtake();
        else if (intakeFwd)  intake.intake();
        else                 intake.stop();

        // Indexer manual advance (g2.LT)
        if (g2.left_trigger > 0.5 && !indexer.isStepping()) indexer.startStep();
        indexer.loop();
        boolean stepTimedOut = indexer.getAndClearTimedOut();
        if (stepTimedOut && Constants.Shooter.JAM_RETRACT_IMMEDIATE) {
            shooter.setRampEngaged(false);
            shooter.idle();
            burstFeeds = 0;
        }

        // Auto-advance on color detect at S1L (when idle)
        if (Constants.Indexer.AUTO_ADVANCE_ON_DETECT && !indexer.isStepping() && recheck == null) {
            if (indexer.detectAtS1L()) {
                // Pause intake briefly during step
                intake.setPaused(true);
                indexer.startStep();
                // simple timer-based resume handled within RecheckTask or next loop after timeout
            }
        }
        // Resume intake after a step ends (simple heuristic)
        if (!indexer.isStepping() && intake.isPaused()) intake.setPaused(false);

        // Auto-apply motif ordering when queue full & motif known
        if (indexer.isQueueFull() && vision.isMotifKnown()) {
            indexer.rotateForMotif(vision.getMotifCode());
        }

        // Shooter control (g2.RT hold-to-fire)
        boolean rt = g2.right_trigger > 0.5;
        if (rt && !rtPrev) {
            shooter.runToTarget();
            spinStartMs = System.currentTimeMillis();
            holdReleaseStartMs = 0;
            burstFeeds = 0;
        }
        if (rt) {
            shooter.runToTarget();
            // Ready after max(SPINUP_WAIT_MS, RAMP_UP_TIME_MS)
            long wait = Math.max(Constants.Shooter.SPINUP_WAIT_MS, Constants.Shooter.RAMP_UP_TIME_MS);
            boolean ready = (System.currentTimeMillis() - spinStartMs) >= wait;
            if (ready && !indexer.isStepping() && burstFeeds < Constants.Shooter.MAX_BURST_FEEDS) {
                shooter.setRampEngaged(true);
                indexer.startStep();
                burstFeeds++;
                // dwell between feeds
                spinStartMs = System.currentTimeMillis() - (Constants.Shooter.SPINUP_WAIT_MS - Constants.Shooter.INTER_SHOT_DWELL_MS);
            }
        } else { // RT released
            shooter.setRampEngaged(false);
            if (holdReleaseStartMs == 0) holdReleaseStartMs = System.currentTimeMillis();
            if (System.currentTimeMillis() - holdReleaseStartMs >= Constants.Shooter.HOLD_AFTER_RELEASE_MS) {
                shooter.idle(); burstFeeds = 0;
            }
        }
        rtPrev = rt;
        shooter.loop();

        // Heading lock (g1.RB hold) and one-shot cardinals (g1 D-pad ←/→)
        boolean rb = g1.right_bumper;
        if (rb && !headingLockHeld) { headingLockHeld = true; lockHeadingRad = heading.getHeadingRad(); }
        else if (!rb && headingLockHeld && !snapActive) { headingLockHeld = false; }

        if (!snapActive) {
            if (g1.dpad_right) { snapTargetRad = nextCardinal(heading.getHeadingRad()); snapActive = true; }
            else if (g1.dpad_left) { snapTargetRad = prevCardinal(heading.getHeadingRad()); snapActive = true; }
        }
        if (snapActive) {
            double errDeg = Math.toDegrees(wrap(snapTargetRad - heading.getHeadingRad()));
            if (Math.abs(errDeg) <= Constants.Drive.SNAP_TOL_DEG) {
                snapActive = false; // reached target; return to normal (or keep RB lock if held)
                if (!rb) headingLockHeld = false;
            } else {
                headingLockHeld = true; lockHeadingRad = snapTargetRad; // drive toward target
            }
        }

        // Drive shaping & transform
        double lx = g1.left_stick_x, ly = -g1.left_stick_y;
        double rx = Constants.Drive.ROTATION_RIGHT_STICK_X ? g1.right_stick_x : g1.left_stick_x;
        lx = Shaping.translate(lx); ly = Shaping.translate(ly); rx = Shaping.rotate(rx);
        double scale = g1.left_bumper ? Constants.Drive.PRECISION_SCALE : 1.0;
        lx *= scale; ly *= scale; if (Constants.Drive.PRECISION_SCALES_ROTATION) rx *= scale;

        // Field-centric transform
        double x = lx, y = ly;
        if (useField) {
            double h = heading.getHeadingRad();
            double cos = Math.cos(-h), sin = Math.sin(-h);
            double tx = x * cos - y * sin;
            double ty = x * sin + y * cos;
            x = tx; y = ty;
        }
        // Rotation sign: RS right => CW on field (=> negative CCW)
        double rotInput = rx * (Constants.Drive.ROTATION_CW_IS_POSITIVE ? -1.0 : 1.0);
        // Heading lock controller overrides rotation
        double rot;
        if (headingLockHeld) {
            double errDeg = Math.toDegrees(wrap(lockHeadingRad - heading.getHeadingRad()));
            double cmdDps = clip(errDeg * Constants.Drive.HEADING_LOCK_KP_PER_DEG,
                    -Constants.Drive.HEADING_LOCK_MAX_ROT_PER_SEC,
                    Constants.Drive.HEADING_LOCK_MAX_ROT_PER_SEC);
            rot = cmdDps / 360.0;
        } else rot = rotInput;

        // Slew limit
        if (Constants.Drive.SLEW_ENABLED) {
            x = slewX.calculate(x); y = slewY.calculate(y); rot = slewR.calculate(rot);
        }
        drive.driveRobotCentric(x, y, rot);

        // Telemetry
        T.t(3, "Alliance", alliance);
        T.t(3, "Motif", vision.isMotifKnown() ? vision.getMotifCode() : "unknown");
        T.t(3, "Queue", String.format("[%s,%s,%s]", indexer.getS0(), indexer.getS1L(), indexer.getS2()));
        T.t(2, "Drive", String.format("FC=%s snap=%s x=%.2f y=%.2f r=%.2f", useField, snapActive, x, y, rot));
        telemetry.update();
    }

    /** Non-blocking "Re-check Slots" routine that runs over multiple loops. */
    private static class RecheckTask {
        private final IndexerSubsystem indexer;
        private final ShooterSubsystem shooter;
        private final IntakeSubsystem intake;
        private final TelemetryUtil T;
        private int stepsDone = 0;
        private boolean started = false;
        public RecheckTask(IndexerSubsystem idx, ShooterSubsystem sh, IntakeSubsystem in, TelemetryUtil T) {
            this.indexer = idx; this.shooter = sh; this.intake = in; this.T = T;
            shooter.setRampEngaged(false); shooter.idle(); intake.setPaused(true);
        }
        public void loop() {
            // start step if not already stepping
            if (!started) { indexer.startStep(); started = true; }
            indexer.loop();
            if (!indexer.isStepping()) {
                // read color at S1L and log overwrite
                boolean det = indexer.detectAtS1L();
                if (det) T.t(3, "Recheck", "Updated S1L: " + indexer.getS1L());
                stepsDone++;
                if (stepsDone >= 3) { intake.setPaused(false); return; }
                // kick next step
                indexer.startStep();
            }
        }
        public boolean isDone() { return stepsDone >= 3 && !indexer.isStepping(); }
    }

    private static double nextCardinal(double rad) {
        double deg = Math.toDegrees(rad);
        if (deg <  45) return Math.toRadians( 90);
        if (deg < 135) return Math.toRadians(180);
        if (deg < 225) return Math.toRadians(270);
        if (deg < 315) return Math.toRadians(  0);
        return Math.toRadians( 90);
    }
    private static double prevCardinal(double rad) {
        double deg = Math.toDegrees(rad);
        if (deg <  90) return Math.toRadians(  0);
        if (deg < 180) return Math.toRadians( 90);
        if (deg < 270) return Math.toRadians(180);
        if (deg < 360) return Math.toRadians(270);
        return Math.toRadians(270);
    }
    private static double wrap(double a) { while (a > Math.PI) a -= 2*Math.PI; while (a < -Math.PI) a += 2*Math.PI; return a; }
    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
