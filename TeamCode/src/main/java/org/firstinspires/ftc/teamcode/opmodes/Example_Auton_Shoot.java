package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;


@Autonomous(name = "Example_Auton_Shoot", group = "RR")
@Disabled
public final class Example_Auton_Shoot extends OpMode {
    // Fields
    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private enum AutoState { START, SHOOT_PRELOAD, AFTER_SHOOT, DONE }
    private AutoState autoState = AutoState.START;

    public void init() {

    }

    @Override
    public void start() {
        shooter = new ShooterSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
        autoState = AutoState.START;
    }

    @Override
    public void loop() {
        switch (autoState) {
            case START:
                // Shoot 3 times, for example
                shooter.startAutonBurst(3);
                autoState = AutoState.SHOOT_PRELOAD;
                break;

            case SHOOT_PRELOAD:
                // Tick the burst helper until it finishes
                if (shooter.updateAutonBurst(indexer)) {
                    autoState = AutoState.AFTER_SHOOT;
                }
                break;

            case AFTER_SHOOT:
                // Shooter is idle, ramp down, shots cleared -> move on
                // drive to next waypoint, etc.
                autoState = AutoState.DONE;
                break;

            case DONE:
                // whatever
                break;
        }

        // If you have a generic shooter.loop() (for ramping power), you can still
        // call it here; updateAutonBurst() uses runToTarget()/idle() to set targetPower.
        shooter.loop();
    }

}
