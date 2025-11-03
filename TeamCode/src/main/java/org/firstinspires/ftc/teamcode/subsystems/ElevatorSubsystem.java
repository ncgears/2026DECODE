package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.util.SimpleTimer;

/** Endgame elevator with NC limit and lockout threshold. */
public class ElevatorSubsystem {
    private final Motor motor;
    private final DigitalChannel limitNC;
    private boolean lockout;
    private final SimpleTimer upRamp = new SimpleTimer();
    private final SimpleTimer downRamp = new SimpleTimer();
    private double commanded = 0.0;

    public ElevatorSubsystem(HardwareMap hw) {
        motor = hw.get(Motor.class, Constants.Elevator.MOTOR);
        limitNC = hw.get(DigitalChannel.class, Constants.Elevator.LIMIT_SW);
        limitNC.setMode(DigitalChannel.Mode.INPUT);
//        motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Call each loop to apply ramps, respect the limit, and update lockout. */
    public void loop() {
        boolean atLimit = !limitNC.getState(); // NC: LOW when pressed
        if (atLimit && commanded > 0) commanded = 0.0; // stop upward motion

        double out;
        if (commanded > 0) {
            double t = Math.min(1.0, upRamp.ms() / (double) Constants.Elevator.RAMP_UP_MS);
            out = t * Constants.Elevator.POWER_UP;
        } else if (commanded < 0) {
            double t = Math.min(1.0, downRamp.ms() / (double) Constants.Elevator.RAMP_DOWN_MS);
            out = t * Constants.Elevator.POWER_DOWN;
        } else out = 0.0;

        motor.set(out);

        // Lockout when encoder beyond threshold
        lockout = motor.getCurrentPosition() > Constants.Elevator.LOCKOUT_ENCODER_THRESHOLD;
    }

    public void commandUp(boolean on)   { if (on) { commanded = +1.0; upRamp.reset(); } else commanded = 0.0; }
    public void commandDown(boolean on) { if (on) { commanded = -1.0; downRamp.reset(); } else commanded = 0.0; }
    public void stop() { commanded = 0.0; motor.set(0.0); }
    public boolean isLockout() { return lockout; }
    public int getEncoder() { return motor.getCurrentPosition(); }
}
