package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.Constants;

/** Intake roller forward/reverse with pause support. */
public class IntakeSubsystem {
    private final Motor motor;
    private boolean paused;
    public IntakeSubsystem(HardwareMap hw) {
        motor = new Motor(hw, Constants.Intake.MOTOR);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }
    public void intake() { if (!paused) motor.set(Constants.Intake.POWER_IN); }
    public void outtake() { motor.set(-Constants.Intake.POWER_OUT); }
    public void stop() { motor.set(0.0); }
    public void setPaused(boolean p) { this.paused = p; if (p) stop(); }
    public boolean isPaused() { return paused; }
}
