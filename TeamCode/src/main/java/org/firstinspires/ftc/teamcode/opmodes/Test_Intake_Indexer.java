package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem.Item;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Test_Intake_Indexer", group = "Diagnostics")
public class Test_Intake_Indexer extends OpMode {

    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;

    // Shared edge-detect state (used in both init_loop and loop)
    private boolean wasStepping   = false;
    private boolean lastLTPressed = false;
    private boolean lastBPressed  = false;

    private boolean lastStepDetectedColor = false;
    private Item lastDetectedItem         = Item.NONE;

    @Override
    public void init() {
        intake  = new IntakeSubsystem(hardwareMap);
        indexer = new IndexerSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        wasStepping          = indexer.isStepping();
        lastLTPressed        = false;
        lastBPressed         = false;
        lastStepDetectedColor = false;
        lastDetectedItem      = Item.NONE;

        telemetry.addLine("Intake_Testing: INIT. Use LT/B to step/clear; color will auto-advance when not full.");
    }

    @Override
    public void init_loop() {
        // Same indexer behavior as in run loop, but without intake control.
        handleIndexer(true, true);
        telemetry.update();
    }

    @Override
    public void start() {
        lastStepDetectedColor = false;
        lastDetectedItem      = indexer.getS1L();
        shooter.resetFireControl();
    }

    @Override
    public void loop() {
        boolean shootTrigger = gamepad2.right_trigger > 0.5;
        boolean autoAdvanceEnabled = !shootTrigger;

        // Indexer behavior
        handleIndexer(false, autoAdvanceEnabled);

        // Intake control (same mapping as Bot_Testing)
        boolean intakeForward = gamepad2.x && !gamepad2.left_bumper;
        boolean outtake       = gamepad2.x &&  gamepad2.left_bumper;

        if (outtake) {
            intake.outtake();
        } else if (intakeForward) {
            intake.intake();
        } else {
            intake.stop();
        }

        // Shooter control + firing on g2.right_trigger
        shooter.handleRightTrigger(shootTrigger, indexer);
        shooter.loop();  // apply soft power ramp and set motors

        telemetry.addLine("== Intake ==");
        telemetry.addData("command",
                outtake ? "OUTTAKE (g2 LB+X)" :
                        intakeForward ? "INTAKE (g2 X)" :
                                "STOP");

        telemetry.update();
    }

    // ------------------------------------------------------------
    // Shared logic for INIT and RUN
    // ------------------------------------------------------------

    private void handleIndexer(boolean inInit, boolean autoAdvanceEnabled) {
        // Keep indexer state machine updated
        indexer.loop();
        boolean isStepping = indexer.isStepping();

        // Edge detection for LT / B
        boolean ltNow        = gamepad2.left_trigger > 0.5;
        boolean ltPressedEdge = ltNow && !lastLTPressed;
        lastLTPressed        = ltNow;

        boolean bNow         = gamepad2.b;
        boolean bPressedEdge = bNow && !lastBPressed;
        lastBPressed         = bNow;

        // Manual stepping with g2.left_trigger (allowed in INIT and RUN)
        if (ltPressedEdge && !isStepping) {
            indexer.startStep();
        }

        // Full re-index / clear with g2.b when idle (INIT and RUN)
        if (bPressedEdge && !isStepping) {
            indexer.clearAll();          // S0/S1L/S2 -> NONE
            lastDetectedItem      = Item.NONE;
            lastStepDetectedColor = false;
        }

        // Edge detect completion of a step
        boolean stepCompletedThisLoop = wasStepping && !isStepping;
        wasStepping = isStepping;

        // ---- AUTO-ADVANCE RULE ----
        // 1) If we are currently stepping, do nothing here.
        // 2) If the queue is FULL (all three slots non-NONE), we DO NOT auto-advance.
        //    Manual LT is still allowed.
        // 3) Otherwise, whenever detectAtS1L() reports a valid color
        //    (exactly one ACTIVE-LOW color line), we update S1L (inside subsystem)
        //    and advance one slot.
        lastStepDetectedColor = false;

        boolean queueFull = isQueueFull();

        if (autoAdvanceEnabled) {
            if (!isStepping && !queueFull) {
                boolean colorPresent = indexer.detectAtS1L();
                if (colorPresent) {
                    lastStepDetectedColor = true;
                    lastDetectedItem = indexer.getS1L();
                    indexer.startStep(); // automatically advance away from the ball
                }
            } else if (!isStepping && queueFull) {
                // Optional: still allow detection to keep S1L's color correct,
                // but do NOT auto-advance.
                indexer.detectAtS1L();
            }
        } else {
            // While shooter is in charge of feeding, we can still sample color
            // for telemetry / debugging, but MUST NOT auto-step.
            if (!isStepping) {
                indexer.detectAtS1L();
            }
        }
        // ---- Telemetry ----
        telemetry.addLine(inInit ? "== INIT / Indexer ==" : "== RUN / Indexer ==");
        telemetry.addData("isStepping", isStepping);
        telemetry.addData("stepCompletedThisLoop", stepCompletedThisLoop);
        telemetry.addData("queueFull", queueFull);
        telemetry.addData("S0",  indexer.getS0());
        telemetry.addData("S1L", indexer.getS1L());
        telemetry.addData("S2",  indexer.getS2());
        telemetry.addData("lastDetectedItem", "%s (updatedThisLoop=%b)",
            lastDetectedItem, lastStepDetectedColor);
        telemetry.addData("colorMode",
            indexer.isRevColorSensorHealthy() ? "REV" : indexer.isDioColorAvailable() ? "DIO (fallback)" : "NO COLOR SENSOR");
    }

    private boolean isQueueFull() {
        return indexer.getS0()  != Item.NONE &&
            indexer.getS1L() != Item.NONE &&
            indexer.getS2()  != Item.NONE;
    }
}
