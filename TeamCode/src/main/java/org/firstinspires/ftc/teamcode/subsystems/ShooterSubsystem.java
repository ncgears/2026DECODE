package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.util.SimpleTimer;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;

/** Dual flywheel shooter with optional soft power ramp and ramp servo control. */
public class ShooterSubsystem {
    private final Motor m1, m2;
    private final Servo ramp;
    private final SimpleTimer powerRampTimer = new SimpleTimer();
    private double targetPower = 0.0;
    private double currentPower = 0.0;

    // High-level fire control state (shared across opmodes)
    private boolean triggerPrev = false;
    private long spinStartMs = 0L;
    private long lastShotCompleteMs = 0L;
    private long holdReleaseStartMs = 0L;
    private boolean shotInProgress = false;
    private boolean shotRequested = false;

    public ShooterSubsystem(HardwareMap hw) {
        m1 = new Motor(hw, Constants.Shooter.MOTOR_1);
        m2 = new Motor(hw, Constants.Shooter.MOTOR_2);
//        m1.setInverted(Constants.Shooter.MOTOR_1_INVERT); // invert motor
//        m2.setInverted(Constants.Shooter.MOTOR_2_INVERT); // invert second motor
        m1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        ramp = hw.get(Servo.class, Constants.Shooter.RAMP_SERVO);
        ramp.setPosition(Constants.Shooter.RAMP_RETRACTED);
        idle();
    }

    /** Reset high-level fire control state (e.g., at OpMode start). */
    public void resetFireControl() {
        triggerPrev = false;
        spinStartMs = 0L;
        lastShotCompleteMs = 0L;
        holdReleaseStartMs = 0L;
        shotInProgress = false;
        shotRequested = false;
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

    /**
     * High-level helper for shooter control tied to a "fire" trigger.
     *
     * Behavior:
     *  - While triggerPressed:
     *      * run flywheels to TARGET_POWER
     *      * after spin-up, start a "shot" by:
     *          - engaging ramp
     *          - starting one indexer step (startStepForShot)
     *      * wait until the indexer finishes that step
     *          - then mark the shot complete, retract ramp
     *      * after a dwell (INTER_SHOT_DWELL_MS) from *completion*,
     *        repeat while trigger is still held.
     *
     *  - On release:
     *      * ramp stays engaged until the current step (if any) finishes,
     *        then retracts.
     *      * flywheels idle after HOLD_AFTER_RELEASE_MS.
     *
     * Call this once per loop from your OpMode, then call shooter.loop().
     */
    public void handleRightTrigger(boolean triggerPressed, IndexerSubsystem indexer) {
        long now = System.currentTimeMillis();
        boolean rt = triggerPressed;

        // Rising edge: user has asked for at least one shot
        if (rt && !triggerPrev) {
            shotRequested = true;

            boolean wasAtTarget = (targetPower >= Constants.Shooter.TARGET_POWER);

            // Ensure we're going to target power
            runToTarget();

            int spinupMs = Math.max(
                    Constants.Shooter.SPINUP_WAIT_MS,
                    Constants.Shooter.RAMP_UP_TIME_MS
            );

            if (!wasAtTarget) {
                // Start a new spin-up
                spinStartMs = now;
            } else if (spinStartMs == 0L) {
                // Treat as already spun-up if we were hot coming in
                spinStartMs = now - spinupMs;
            }

            // New trigger hold: dwell timing for this sequence
            lastShotCompleteMs = 0L;
        }

        int spinupMs = Math.max(
                Constants.Shooter.SPINUP_WAIT_MS,
                Constants.Shooter.RAMP_UP_TIME_MS
        );
        boolean spunUp = (spinStartMs > 0L) && ((now - spinStartMs) >= spinupMs);

        boolean indexerStepping = indexer.isStepping();

        // Track when an in-progress shot finishes
        if (shotInProgress && !indexerStepping) {
            shotInProgress = false;
            lastShotCompleteMs = now;
        }

        boolean dwellOk =
                (lastShotCompleteMs == 0L) ||
                        ((now - lastShotCompleteMs) >= Constants.Shooter.INTER_SHOT_DWELL_MS);

        // Two notions of "want shots":
        boolean wantShotStream = rt;                         // holding trigger = stream mode
        boolean wantAnyShot    = wantShotStream || shotRequested; // tap or hold

        // ---- Start a shot if conditions are right ----
        if (!shotInProgress && wantAnyShot && spunUp && dwellOk && !indexerStepping) {
            // This covers:
            //  - tap while idle (latched until spin-up complete)
            //  - tap while already hot (immediate)
            //  - hold for multiple shots (RT stays true)
            indexer.startStepForShot();
            shotInProgress = true;
            shotRequested  = false;  // consumed the latched request
        }

        // ---- Ramp control ----
        if (shotInProgress) {
            // During a feed step, ramp must be up
            setRampEngaged(true);
        } else if (wantShotStream && spunUp) {
            // Between shots in a stream: keep ramp up as long as trigger is held
            setRampEngaged(true);
        } else {
            // No active shot, no stream mode => ramp down
            setRampEngaged(false);
        }

        // ---- Shooter power / idle control ----
        boolean fireDemand = wantAnyShot || shotInProgress;

        if (fireDemand) {
            // Either we have a pending/active shot or a held trigger:
            // keep spinning up / at speed, and do NOT start idle timer
            runToTarget();
            holdReleaseStartMs = 0L;
        } else {
            // No pending shots and trigger not held: maybe spin down
            if (targetPower > Constants.Shooter.IDLE_POWER) {
                if (holdReleaseStartMs == 0L) {
                    holdReleaseStartMs = now;
                }
                if (now - holdReleaseStartMs >= Constants.Shooter.HOLD_AFTER_RELEASE_MS) {
                    idle();
                    spinStartMs = 0L;  // force fresh spin-up next time
                }
            } else {
                holdReleaseStartMs = 0L;
                spinStartMs = 0L;
            }
        }

        triggerPrev = rt;
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
