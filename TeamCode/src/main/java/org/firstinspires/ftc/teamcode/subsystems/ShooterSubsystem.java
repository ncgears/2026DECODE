package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.util.SimpleTimer;

/** Dual flywheel shooter with optional soft power ramp and ramp servo control. */
public class ShooterSubsystem {
    private final Motor m1, m2;
    private final Servo ramp;
    private final SimpleTimer powerRampTimer = new SimpleTimer();
    private double targetPower = 0.0;
    private double currentPower = 0.0;

    public ShooterSubsystem(HardwareMap hw) {
        m1 = hw.get(Motor.class, Constants.Shooter.MOTOR_1);
        m2 = hw.get(Motor.class, Constants.Shooter.MOTOR_2);
//        m1.setInverted(Constants.Shooter.MOTOR_1_INVERT); // invert motor
//        m2.setInverted(Constants.Shooter.MOTOR_2_INVERT); // invert second motor
        m1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        ramp = hw.get(Servo.class, Constants.Shooter.RAMP_SERVO);
        ramp.setPosition(Constants.Shooter.RAMP_RETRACTED);
        idle();
    }

    /** Hold flywheels at idle power. */
    public void idle() { setTargetPower(Constants.Shooter.IDLE_POWER); }

    /** Stop flywheels completely. */
    public void stop() { setTargetPower(0.0); }

    /** Run flywheels to target shooting power. */
    public void runToTarget() { setTargetPower(Constants.Shooter.TARGET_POWER); }

    /** Engage or retract the feed ramp. */
    public void setRampEngaged(boolean engaged) {
        ramp.setPosition(engaged ? Constants.Shooter.RAMP_ENGAGED : Constants.Shooter.RAMP_RETRACTED);
    }

    /** Call each loop to apply soft ramping and set motor power. */
    public void loop() {
        int rampMs = Constants.Shooter.RAMP_UP_TIME_MS;
        if (rampMs <= 0) currentPower = targetPower;
        else {
            double t = Math.min(1.0, powerRampTimer.ms() / (double) rampMs);
            currentPower = (1.0 - t) * currentPower + t * targetPower;
        }
        m1.set(currentPower * (Constants.Shooter.MOTOR_1_INVERT ? -1.0 : 1.0));
        m2.set(currentPower * (Constants.Shooter.MOTOR_2_INVERT ? -1.0 : 1.0));
    }

    public void setTargetPower(double p) {
        targetPower = clip(p, 0.0, 1.0);
        powerRampTimer.reset();
    }
    public double getCurrentPower() { return currentPower; }

    private static double clip(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
