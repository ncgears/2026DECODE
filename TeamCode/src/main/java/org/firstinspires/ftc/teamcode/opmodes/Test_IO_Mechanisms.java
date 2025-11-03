package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

/**
 * Test_IO_Mechanisms
 * Diagnostics TeleOp for digital IO + mechanisms (no drive).
 *
 * - Telemetry: shows all digital inputs with raw level and "asserted" state.
 * - Indexer CR servo: manual forward/reverse run.
 * - Ramp servo: engage/retract.
 * - Intake + Shooter motors: run/idle/stop.
 *
 * Controls (gamepad2):
 *   Intake:
 *     - X (hold)            => intake forward (Constants.Intake.POWER_IN)
 *     - LB + X (hold)       => outtake (Constants.Intake.POWER_OUT)
 *     - release             => stop
 *
 *   Indexer CR servo:
 *     - D-pad RIGHT (hold)  => forward (Constants.Indexer.POWER_FWD * sign)
 *     - D-pad LEFT  (hold)  => reverse
 *     - release             => stop
 *
 *   Ramp servo:
 *     - A                   => engage  (Constants.Shooter.RAMP_ENGAGED)
 *     - B                   => retract (Constants.Shooter.RAMP_RETRACTED)
 *
 *   Shooter:
 *     - RT > 0.5            => TARGET power (Constants.Shooter.TARGET_POWER) both motors
 *     - LT > 0.5            => IDLE power   (Constants.Shooter.IDLE_POWER)   both motors
 *     - Y                   => STOP (0.0)   both motors
 *
 * Panic (either pad):
 *   - BACK                  => stop intake/indexer/shooter; retract ramp
 */
@TeleOp(name = "Test_IO_Mechanisms", group = "Diagnostics")
public class Test_IO_Mechanisms extends OpMode {

    // Motors (SDK DcMotor is sufficient for testing)
    private DcMotor intake;
    private DcMotor shooter1, shooter2;

    // Servos
    private CRServo indexer;
    private Servo   ramp;

    // Digital inputs
    private DigitalChannel diIndexSlot;   // NC (LOW when pressed)
    private DigitalChannel diIndexPurple; // PNP (HIGH when purple)
    private DigitalChannel diIndexGreen;  // PNP (HIGH when green)
    private DigitalChannel diElevLim;     // NC (LOW when pressed)
    private DigitalChannel diFlagA;       // asserted LOW
    private DigitalChannel diFlagB;       // asserted LOW
    private DigitalChannel diAutoA;       // asserted LOW
    private DigitalChannel diAutoB;       // asserted LOW

    // Cached forward power for indexer with direction sign
    private double indexerFwdPower;

    @Override
    public void init() {
        HardwareMap hw = hardwareMap;

        // --- Motors ---
        intake   = hw.get(DcMotor.class, Constants.Intake.MOTOR);
        shooter1 = hw.get(DcMotor.class, Constants.Shooter.MOTOR_1);
        shooter2 = hw.get(DcMotor.class, Constants.Shooter.MOTOR_2);

        // Let mechanisms coast when stopped (safe for bench testing)
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // --- Servos ---
        indexer = hw.get(CRServo.class, Constants.Indexer.SERVO);
        ramp    = hw.get(Servo.class,   Constants.Shooter.RAMP_SERVO);
        ramp.setPosition(Constants.Shooter.RAMP_RETRACTED);

        indexerFwdPower = Constants.Indexer.POWER_FWD *
                (Constants.Indexer.DIR_FORWARD_IS_POSITIVE ? +1.0 : -1.0);

        // --- Digital Inputs (all INPUT mode) ---
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

        telemetry.addLine("Diagnostics ready: test digital IO + indexer/ramp/intake/shooter");
    }

    @Override
    public void loop() {
        // -----------------------------
        // Controls (gamepad2)
        // -----------------------------

        // Panic stop + retract ramp
        if (gamepad1.back || gamepad2.back) {
            stopAll();
        }

        // Intake
        boolean intakeFwd = gamepad2.x && !gamepad2.left_bumper;
        boolean intakeRev = gamepad2.x &&  gamepad2.left_bumper;
        if      (intakeRev) intake.setPower(-Constants.Intake.POWER_OUT);
        else if (intakeFwd) intake.setPower(+Constants.Intake.POWER_IN);
        else                intake.setPower(0.0);

        // Indexer CR servo manual
        double idxPower = 0.0;
        if (gamepad2.dpad_right) idxPower =  indexerFwdPower;
        else if (gamepad2.dpad_left) idxPower = -indexerFwdPower;
        indexer.setPower(idxPower);

        // Ramp servo toggle
        if (gamepad2.a) ramp.setPosition(Constants.Shooter.RAMP_ENGAGED);
        if (gamepad2.b) ramp.setPosition(Constants.Shooter.RAMP_RETRACTED);

        // Shooter motors
        boolean rt = gamepad2.right_trigger > 0.5;
        boolean lt = gamepad2.left_trigger  > 0.5;
        if (gamepad2.y) { // STOP
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

        // -----------------------------
        // Telemetry (digital IO + outputs)
        // -----------------------------
        telemetry.addLine("== Digital Inputs ==");
        // Helper to print raw (HIGH/LOW) and asserted status
        telemetry.addData("indexslot (NC)", fmt(diIndexSlot), assertedNC(diIndexSlot) ? "PRESSED" : "released");
        telemetry.addData("indexpurple (PNP)", fmt(diIndexPurple), assertedPNP(diIndexPurple) ? "DETECTED" : "no");
        telemetry.addData("indexgreen  (PNP)", fmt(diIndexGreen),  assertedPNP(diIndexGreen)  ? "DETECTED" : "no");
        telemetry.addData("elevlim   (NC)", fmt(diElevLim), assertedNC(diElevLim) ? "PRESSED" : "released");

        telemetry.addData("flag sw a (LOW=assert)", fmt(diFlagA), assertedLOW(diFlagA) ? "ASSERTED" : "open");
        telemetry.addData("flag sw b (LOW=assert)", fmt(diFlagB), assertedLOW(diFlagB) ? "ASSERTED" : "open");
        telemetry.addData("auto sw a (LOW=assert)", fmt(diAutoA), assertedLOW(diAutoA) ? "ASSERTED" : "open");
        telemetry.addData("auto sw b (LOW=assert)", fmt(diAutoB), assertedLOW(diAutoB) ? "ASSERTED" : "open");

        telemetry.addLine("== Outputs ==");
        telemetry.addData("Indexer CR", "%.2f", idxPower);
        telemetry.addData("Ramp pos", "%.2f", ramp.getPosition());
        telemetry.addData("Intake", "%.2f", intake.getPower());
        telemetry.addData("Shooter1", "%.2f", shooter1.getPower());
        telemetry.addData("Shooter2", "%.2f", shooter2.getPower());

        telemetry.addLine("g2: X=Intake  LB+X=Outtake | D-pad R/L=Indexer +/− | A=RampEngage  B=RampRetract | RT=ShooterTarget  LT=ShooterIdle  Y=ShooterStop | BACK=Panic");
        telemetry.update();
    }

    private void stopAll() {
        intake.setPower(0.0);
        indexer.setPower(0.0);
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);
        ramp.setPosition(Constants.Shooter.RAMP_RETRACTED);
    }

    // ---- Telemetry helpers ----
    private String fmt(DigitalChannel ch) { return ch.getState() ? "HIGH" : "LOW"; }
    private boolean assertedNC(DigitalChannel ch)   { return !ch.getState(); } // NC switches: LOW when pressed
    private boolean assertedPNP(DigitalChannel ch)  { return  ch.getState(); } // PNP color lines: HIGH when detected
    private boolean assertedLOW(DigitalChannel ch)  { return !ch.getState(); } // Flag/Auto: asserted when LOW
}
