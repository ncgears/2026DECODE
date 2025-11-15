package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.Item;

/**
 * Intake_Testing
 *
 * Diagnostics TeleOp focused on:
 *   - IntakeSubsystem behavior (intake / outtake / stop)
 *   - IndexerStepper behavior using slot limit switch (indexslot)
 *   - Discrete color DIO lines (indexpurple / indexgreen), wired ACTIVE-LOW
 *
 * Behavior:
 *   - On startup, performs a single "homing" step: run forward until the NC slot switch
 *     transitions from LOW (pressed) to HIGH (released), via IndexerSubsystem.startStep()/loop().
 *   - While running:
 *       gamepad2.x (hold)          => intake forward
 *       gamepad2.left_bumper + x   => outtake
 *       gamepad2.left_trigger>0.5  => advance indexer one slot (when READY)
 *       gamepad2.y (hold)          => clear queue (ALL NONE)
 *
 *   - After each completed step (including homing), samples the color lines at S1L via
 *     IndexerSubsystem.detectAtS1L() using ACTIVE-LOW semantics:
 *       indexpurple/indexgreen: HIGH = idle, LOW = color present
 */
@TeleOp(name = "Intake_Testing", group = "Diagnostics")
public class Intake_Testing extends OpMode {

    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;

    private enum IndexerMode {
        HOMING_REQUESTED,
        HOMING_IN_PROGRESS,
        READY
    }

    private IndexerMode indexerMode = IndexerMode.HOMING_REQUESTED;

    private boolean wasStepping = false;
    private boolean lastStepDetectedColor = false;
    private Item lastDetectedItem = Item.NONE;

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);

        indexerMode = IndexerMode.HOMING_REQUESTED;
        wasStepping = false;
        lastStepDetectedColor = false;
        lastDetectedItem = Item.NONE;

        telemetry.addLine("Intake_Testing: init complete. Waiting for start.");
    }

    @Override
    public void loop() {
        // Keep indexer internal state machine updated
        indexer.loop();
        boolean isStepping = indexer.isStepping();

        // ---------- Indexer homing / stepping ----------
        switch (indexerMode) {
            case HOMING_REQUESTED:
                // On the first loop after init, request exactly one step if idle
                if (!isStepping) {
                    indexer.startStep();
                    indexerMode = IndexerMode.HOMING_IN_PROGRESS;
                }
                break;

            case HOMING_IN_PROGRESS:
                // Once the homing step finishes (slot switch HIGH edge seen), we're READY
                if (!isStepping) {
                    indexerMode = IndexerMode.READY;
                }
                break;

            case READY:
                // Manual advance: LT > 0.5 requests one additional step when idle
                boolean advanceStep = gamepad2.left_trigger > 0.5;
                if (advanceStep && !isStepping) {
                    indexer.startStep();
                }
                break;
        }

        // Edge-detect completion of a step: wasStepping -> !isStepping
        boolean stepCompletedThisLoop = wasStepping && !isStepping;
        wasStepping = isStepping;

        // After each completed step, sample color lines at S1L via subsystem
        if (stepCompletedThisLoop) {
            lastStepDetectedColor = indexer.detectAtS1L();
            lastDetectedItem = lastStepDetectedColor ? indexer.getS1L() : Item.NONE;
        }

        // ---------- Intake control ----------
        boolean intakeForward = gamepad2.x && !gamepad2.left_bumper;
        boolean outtake = gamepad2.x && gamepad2.left_bumper;

        if (outtake) {
            intake.outtake();
        } else if (intakeForward) {
            intake.intake();
        } else {
            intake.stop();
        }

        // Clear the indexer queue with Y (useful to reset state in testing)
        if (gamepad2.y) {
            indexer.clearAll();
        }

        // ---------- Telemetry ----------
        telemetry.addLine("== Indexer ==");
        telemetry.addData("mode", indexerMode);
        telemetry.addData("isStepping", isStepping);
        telemetry.addData("stepCompletedThisLoop", stepCompletedThisLoop);
        telemetry.addData("lastStepColorUpdate", "%s (updated=%b)",
                lastDetectedItem, lastStepDetectedColor);

        telemetry.addLine("Queue S0 / S1L / S2");
        telemetry.addData("S0",  indexer.getS0());
        telemetry.addData("S1L", indexer.getS1L());
        telemetry.addData("S2",  indexer.getS2());

        telemetry.addLine("== Intake ==");
        telemetry.addData("command",
                outtake ? "OUTTAKE (g2 LB+X)" :
                        intakeForward ? "INTAKE (g2 X)" :
                                "STOP");

        telemetry.update();
    }
}
