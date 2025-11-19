
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HeadingProvider;
import org.firstinspires.ftc.teamcode.subsystems.IMUHeading;
import org.firstinspires.ftc.teamcode.util.Shaping;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

/**
 * Bot_Testing
 * Diagnostics + Drive:
 * - Real-time telemetry for all digital inputs
 * - Manual control of indexer CR servo and ramp servo
 * - Manual test of intake and shooter motors
 * - Full drive (robot-centric or field-centric) with shaping, precision scaling, optional slew
 *
 * Controls:
 *   Gamepad1 (Drive)
 *     - LS (x/y): translation
 *     - RS.x (or LS.x per constant): rotation
 *     - LB (hold): precision scale (Constants.Drive.PRECISION_SCALE), scales rotation if enabled
 *     - Y: toggle field-centric
 *     - BACK: zero IMU heading
 *
 *   Gamepad2 (Mechanisms)
 *     Intake:
 *       - X (hold)           => intake forward (Constants.Intake.POWER_IN)
 *       - LB + X (hold)      => outtake (Constants.Intake.POWER_OUT)
 *       - release            => stop
 *
 *     Indexer CR servo:
 *       - D-pad RIGHT (hold) => forward (Constants.Indexer.POWER_FWD * sign)
 *       - D-pad LEFT  (hold) => reverse
 *
 *     Ramp servo:
 *       - A                  => engage  (Constants.Shooter.RAMP_ENGAGED)
 *       - B                  => retract (Constants.Shooter.RAMP_RETRACTED)
 *
 *     Shooter:
 *       - RT > 0.5           => TARGET power (Constants.Shooter.TARGET_POWER) both motors
 *       - LT > 0.5           => IDLE power   (Constants.Shooter.IDLE_POWER)   both motors
 *       - Y                  => STOP (0.0)   both motors
 *
 * Panic:
 *   - BACK on either pad     => stop intake/indexer/shooter, retract ramp (drive keeps obeying sticks)
 */
@TeleOp(name = "Bot_Testing", group = "Diagnostics")
@Disabled
public class Bot_Testing extends OpMode {

    // ---- Drive ----
    private DriveSubsystem drive;
    private HeadingProvider imuHeading;
    private boolean useField = Constants.Drive.FIELD_CENTRIC_DEFAULT;

    private SlewRateLimiter slewX, slewY, slewR;

    // ---- Mechanisms ----
    private DcMotor intake;
    private DcMotor shooter1, shooter2;
    private CRServo indexer;
    private Servo   ramp;

    // ---- Digital Inputs ----
    private DigitalChannel diIndexSlot;   // NC (LOW when pressed)
    private DigitalChannel diIndexPurple; // PNP (HIGH when purple)
    private DigitalChannel diIndexGreen;  // PNP (HIGH when green)
    private DigitalChannel diElevLim;     // NC (LOW when pressed)
    private DigitalChannel diFlagA;       // asserted when LOW
    private DigitalChannel diFlagB;       // asserted when LOW
    private DigitalChannel diAutoA;       // asserted when LOW
    private DigitalChannel diAutoB;       // asserted when LOW

    private double indexerFwdPower;

    @Override
    public void init() {
        // ---- Drive ----
        drive = new DriveSubsystem(hardwareMap);
        imuHeading = new IMUHeading(hardwareMap);

        slewX = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewY = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewR = new SlewRateLimiter(Constants.Drive.SLEW_ROTATION);

        // ---- Mechanisms ----
        HardwareMap hw = hardwareMap;
        intake   = hw.get(DcMotor.class, Constants.Intake.MOTOR);
        shooter1 = hw.get(DcMotor.class, Constants.Shooter.MOTOR_1);
        shooter2 = hw.get(DcMotor.class, Constants.Shooter.MOTOR_2);

        // Coast during diagnostics unless you prefer brake
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        indexer = hw.get(CRServo.class, Constants.Indexer.SERVO);
        ramp    = hw.get(Servo.class,   Constants.Shooter.RAMP_SERVO);
        ramp.setPosition(Constants.Shooter.RAMP_RETRACTED);

        indexerFwdPower = Constants.Indexer.POWER_FWD *
                (Constants.Indexer.DIR_FORWARD_IS_POSITIVE ? +1.0 : -1.0);

        // ---- Digital Inputs ----
        diIndexSlot   = hw.get(DigitalChannel.class, Constants.Indexer.SLOT_LIMIT);
        diIndexPurple = hw.get(DigitalChannel.class, Constants.Indexer.PURPLE_DI);
        diIndexGreen  = hw.get(DigitalChannel.class, Constants.Indexer.GREEN_DI);
        diElevLim     = hw.get(DigitalChannel.class, Constants.Elevator.LIMIT_SW);
        diFlagA       = hw.get(DigitalChannel.class, Constants.Digital.FLAG_A);
        diFlagB       = hw.get(DigitalChannel.class, Constants.Digital.FLAG_B);
        diAutoA       = hw.get(DigitalChannel.class, Constants.Digital.AUTO_A);
        diAutoB       = hw.get(DigitalChannel.class, Constants.Digital.AUTO_B);

        diIndexSlot.setMode(DigitalChannel.Mode.INPUT);
        diIndexPurple.setMode(DigitalChannel.Mode.INPUT);
        diIndexGreen.setMode(DigitalChannel.Mode.INPUT);
        diElevLim.setMode(DigitalChannel.Mode.INPUT);
        diFlagA.setMode(DigitalChannel.Mode.INPUT);
        diFlagB.setMode(DigitalChannel.Mode.INPUT);
        diAutoA.setMode(DigitalChannel.Mode.INPUT);
        diAutoB.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Bot_Testing ready: drive + diagnostics.");
    }

    @Override
    public void loop() {
        // ===== PANIC =====
        if (gamepad1.back || gamepad2.back) stopMechanisms();

        // ===== DRIVE (Gamepad1) =====
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;

        double rawRxRight = gamepad1.right_stick_x;
        double rawRxLeft  = gamepad1.left_stick_x;
        double rx = Constants.Drive.ROTATION_RIGHT_STICK_X ? rawRxRight : rawRxLeft;

        // shaping
        lx = Shaping.translate(lx);
        ly = Shaping.translate(ly);
        rx = Shaping.rotate(rx);

        // precision scaling
        double scale = gamepad1.left_bumper ? Constants.Drive.PRECISION_SCALE : 1.0;
        lx *= scale; ly *= scale; if (Constants.Drive.PRECISION_SCALES_ROTATION) rx *= scale;

        // toggle field-centric
        if (gamepad1.y) useField = !useField;

        // field-centric transform
        double x = lx, y = ly;
        if (useField) {
            double h = imuHeading.getHeadingRad();
            double cos = Math.cos(-h), sin = Math.sin(-h);
            double tx = x * cos - y * sin;
            double ty = x * sin + y * cos;
            x = tx; y = ty;
        }

        // zero heading
        if (gamepad1.back) imuHeading.zeroHeading();

        // sign convention for rotation
        double rot = rx * (Constants.Drive.ROTATION_CW_IS_POSITIVE ? -1.0 : 1.0);

        // optional slew
        if (Constants.Drive.SLEW_ENABLED) {
            x   = slewX.calculate(x);
            y   = slewY.calculate(y);
            rot = slewR.calculate(rot);
        }

        drive.driveRobotCentric(x, y, rot);

        // ===== MECHANISMS (Gamepad2) =====
        // Intake
        boolean intakeFwd = gamepad2.x && !gamepad2.left_bumper;
        boolean intakeRev = gamepad2.x &&  gamepad2.left_bumper;
        if      (intakeRev) intake.setPower(-Constants.Intake.POWER_OUT);
        else if (intakeFwd) intake.setPower(+Constants.Intake.POWER_IN);
        else                intake.setPower(0.0);

        // Indexer CR servo manual
        double idxPower = 0.0;
        if (gamepad2.dpad_right)      idxPower =  indexerFwdPower;
        else if (gamepad2.dpad_left)  idxPower = -indexerFwdPower;
        indexer.setPower(idxPower);

        // Ramp servo
        if (gamepad2.a) ramp.setPosition(Constants.Shooter.RAMP_ENGAGED);
        if (gamepad2.b) ramp.setPosition(Constants.Shooter.RAMP_RETRACTED);

        // Shooter
        boolean rt = gamepad2.right_trigger > 0.5;
        boolean lt = gamepad2.left_trigger  > 0.5;
        if (gamepad2.y) {
            shooter1.setPower(0.0);
            shooter2.setPower(0.0);
        } else if (rt) { // TARGET
            shooter1.setPower(Constants.Shooter.TARGET_POWER * (Constants.Shooter.MOTOR_1_INVERT ? -1.0 : 1.0));
            shooter2.setPower(Constants.Shooter.TARGET_POWER * (Constants.Shooter.MOTOR_2_INVERT ? -1.0 : 1.0));
        } else if (lt) { // IDLE
            shooter1.setPower(Constants.Shooter.IDLE_POWER * (Constants.Shooter.MOTOR_1_INVERT ? -1.0 : 1.0));
            shooter2.setPower(Constants.Shooter.IDLE_POWER * (Constants.Shooter.MOTOR_2_INVERT ? -1.0 : 1.0));
        } else {
            // leave last power as-is to allow listening to spin behavior
        }

        // ===== TELEMETRY =====
        telemetry.addLine("== Digital Inputs ==");
        telemetry.addData("indexslot (NC)",   fmt(diIndexSlot),   assertedNC(diIndexSlot) ? "PRESSED" : "released");
        telemetry.addData("indexpurple (PNP)",fmt(diIndexPurple), assertedPNP(diIndexPurple) ? "DETECTED" : "no");
        telemetry.addData("indexgreen  (PNP)",fmt(diIndexGreen),  assertedPNP(diIndexGreen)  ? "DETECTED" : "no");
        telemetry.addData("elevlim   (NC)",   fmt(diElevLim),     assertedNC(diElevLim) ? "PRESSED" : "released");
        telemetry.addData("flag sw a (LOW=assert)", fmt(diFlagA), assertedLOW(diFlagA) ? "ASSERTED" : "open");
        telemetry.addData("flag sw b (LOW=assert)", fmt(diFlagB), assertedLOW(diFlagB) ? "ASSERTED" : "open");
        telemetry.addData("auto sw a (LOW=assert)", fmt(diAutoA), assertedLOW(diAutoA) ? "ASSERTED" : "open");
        telemetry.addData("auto sw b (LOW=assert)", fmt(diAutoB), assertedLOW(diAutoB) ? "ASSERTED" : "open");

        telemetry.addLine("== Outputs ==");
        telemetry.addData("Indexer CR", "%.2f", idxPower);
        telemetry.addData("Ramp pos",   "%.2f", ramp.getPosition());
        telemetry.addData("Intake",     "%.2f", intake.getPower());
        telemetry.addData("Shooter1",   "%.2f", shooter1.getPower());
        telemetry.addData("Shooter2",   "%.2f", shooter2.getPower());

        telemetry.addLine("g1: LS xy drive, RS.x rot | LB precision | Y FC toggle | BACK zero IMU");
        telemetry.addLine("g2: X intake, LB+X outtake | D-pad R/L indexer +/- | A/B ramp | RT target, LT idle, Y stop | BACK panic");
        telemetry.update();
    }

    private void stopMechanisms() {
        intake.setPower(0.0);
        indexer.setPower(0.0);
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);
        ramp.setPosition(Constants.Shooter.RAMP_RETRACTED);
    }

    // ---- Telemetry helpers ----
    private String fmt(DigitalChannel ch) { return ch.getState() ? "HIGH" : "LOW"; }
    private boolean assertedNC(DigitalChannel ch)   { return !ch.getState(); } // NC: LOW when pressed
    private boolean assertedPNP(DigitalChannel ch)  { return  ch.getState(); } // PNP color lines: HIGH when detected
    private boolean assertedLOW(DigitalChannel ch)  { return !ch.getState(); } // Flag/Auto asserted when LOW
}
