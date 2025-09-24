package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.Constants;

/** Intake roller forward/reverse with pause support. */
public class IntakeSubsystem {
    private final DcMotorEx motor;
    private boolean paused;
    public IntakeSubsystem(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, Constants.Intake.MOTOR);
        motor.setZeroPowerBehavior(com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void intake() { if (!paused) motor.setPower(Constants.Intake.POWER_IN); }
    public void outtake() { motor.setPower(-Constants.Intake.POWER_OUT); }
    public void stop() { motor.setPower(0.0); }
    public void setPaused(boolean p) { this.paused = p; if (p) stop(); }
    public boolean isPaused() { return paused; }
}
