package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSelector.AutoMode;
import org.firstinspires.ftc.teamcode.vision.AprilTagVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * Main DECODE autonomous using RR 1.0, AllianceDetector, and AutoSelector.
 *
 * Sequence:
 *   a) BaseAutoRR seeds RR + Pinpoint with startPose.
 *   b) Determine motif from motifcam ("GPP","PGP","PPG", or "NONE").
 *   c) Rotate the indexer queue for motif shooting order.
 *   d) Move to alliance/mode-specific shooting pose.
 *   e) Shoot 3 shots using ShooterSubsystem auton burst helper.
 *   f) For up to three stacks:
 *        - Drive to stack with intake running.
 *        - Drive back to shooting pose.
 *        - Rotate queue for motif.
 *        - Shoot 3 more.
 */
@Config
@Autonomous(name = "Auto_DECODE_NoDrive", group = "RR")
public final class Auto_DECODE_NoDrive extends BaseAutoRR {
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    private ShooterSubsystem shooter;
    private boolean subsystemsInitialized = false;

    // For preload telemetry/debug
    private IndexerSubsystem.Item lastDetectedItem   = IndexerSubsystem.Item.NONE;
    private boolean lastStepDetectedColor = false;

    // ---- Tunable field coordinates (inches, radians) ----
    // Shooting pose per AutoMode. Replace with real DECODE coordinates.
    public static double RED1_SHOOT_X   =  18.0;
    public static double RED1_SHOOT_Y   = -60.0;
    public static double RED1_SHOOT_HE  =  0.0;

    public static double RED2_SHOOT_X   = -18.0;
    public static double RED2_SHOOT_Y   = -60.0;
    public static double RED2_SHOOT_HE  =  0.0;

    public static double BLUE1_SHOOT_X  =  18.0;
    public static double BLUE1_SHOOT_Y  =  60.0;
    public static double BLUE1_SHOOT_HE = Math.PI;   // facing "back" toward blue goal

    public static double BLUE2_SHOOT_X  = -18.0;
    public static double BLUE2_SHOOT_Y  =  60.0;
    public static double BLUE2_SHOOT_HE = Math.PI;

    // ---- Spike stack geometry (DECODE) ----
    // Assumption for these defaults:
    //  - Goal walls are at |Y| ~= 72".
    //  - Spike centers are 24" in from the goal wall → |Y| = 48".
    //  - Along the goal wall, the centers are 60", 84", 108" from the field end.
    // You will tune these per-auto in Dashboard, but the pattern is correct.

    // RED1: “goal wall” at negative Y, alliance on the red side.
    public static double RED1_SPIKE_Y = -48.0;          // 24" in from red goal wall
    public static double RED1_SPIKE1_X =  60.0;         // PPG spike (closest to goal)
    public static double RED1_SPIKE2_X =  84.0;         // PGP
    public static double RED1_SPIKE3_X = 108.0;         // GPP

    // RED2: mirrored on the other half of the red wall.
    public static double RED2_SPIKE_Y = -48.0;
    public static double RED2_SPIKE1_X = -60.0;
    public static double RED2_SPIKE2_X = -84.0;
    public static double RED2_SPIKE3_X = -108.0;

    // BLUE1: goal wall at positive Y, mirrored geometry.
    public static double BLUE1_SPIKE_Y =  48.0;         // 24" in from blue goal wall
    public static double BLUE1_SPIKE1_X =  60.0;        // PPG
    public static double BLUE1_SPIKE2_X =  84.0;        // PGP
    public static double BLUE1_SPIKE3_X = 108.0;        // GPP

    // BLUE2: mirrored on the other half of the blue wall.
    public static double BLUE2_SPIKE_Y =  48.0;
    public static double BLUE2_SPIKE1_X = -60.0;
    public static double BLUE2_SPIKE2_X = -84.0;
    public static double BLUE2_SPIKE3_X = -108.0;

    // How far “in front of” a spike we stage before driving straight into it.
    public static double SPIKE_APPROACH_OFFSET = 8.0;   // inches along +Y/-Y direction

    // How many notes to shoot per cycle.
    public static int SHOTS_PER_CYCLE = 3;
    public static int MAX_STACKS      = 1;

    @Override
    protected Action buildRoutine(MecanumDrive drive,
                                  Alliance alliance,
                                  AutoMode autoMode,
                                  Pose2d startPose) {
        // Make sure we have subsystems, whether or not initLoop ran
        ensureSubsystems();

        // ---- Subsystems used during this auto ----
        final IntakeSubsystem  intake  = this.intake;
        final IndexerSubsystem indexer = this.indexer;
        final ShooterSubsystem shooter = this.shooter;

        boolean preferRed = (alliance == Alliance.RED);

        // Put shooter/ramp in a sane starting state.
        shooter.idle();
        shooter.setRampEngaged(false);
        // Optionally pre-populate the indexer queue here if you want software to
        // know about preloads before the first shot.

        // ---- (b) Motif was already detected/locked by BaseAutoRR ----
        final String motif = getMotifCode();

        // ---- (c) Rotate queue for shooting order ----
        indexer.rotateForMotif(motif);

        playAudio(String.format("%s", alliance.toString()),500);
        playAudio(String.format("%s", autoMode.toString()),500);
        playAudio(String.format("%s", motif),500);
        T.t(1, "Motif code", motif);
        telemetry.update();

        // ---- (d) Resolve shooting & stack poses for this alliance/mode ----
        Pose2d shootPose = getShootPoseFor(alliance, autoMode);
        Pose2d[] stacks  = getStackPosesFor(alliance, autoMode);

        // ---- Build the main RR action sequence ----
        List<Action> sequence = new ArrayList<>();

        // d) Move from starting pose to shooting pose.
        /*
        Action toFirstShoot = drive.actionBuilder(startPose)
                .lineToX(shootPose.position.x)
                .lineToY(shootPose.position.y)
                // if you later add turn/heading support:
                // .turn(shootPose.heading - startPose.heading)
                .build();
        sequence.add(toFirstShoot);
        */

        // e) Shoot 3 preloads at first shooting pose.
//        sequence.add(makeShootBurstAction(
//                shooter, indexer, SHOTS_PER_CYCLE, alliance, autoMode, "preload"));

        // After completing the preload burst and post-burst hold,
        // explicitly idle the shooter and drop the ramp as a safety reset.
        sequence.add(new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                shooter.idle();
                shooter.setRampEngaged(false);
                packet.put("Shooter", "Idle after preload burst");
                return false; // one-shot action
            }
        });

        // f/g/e) For each stack: go collect, come back, rotate for motif, shoot.
        int stackCount = Math.min(MAX_STACKS, stacks.length);
        Pose2d currentPose = shootPose;

        return new SequentialAction(sequence.toArray(new Action[0]));
    }

    /**
     * Map alliance + auto mode to a shooting pose.
     * Replace placeholders with real DECODE coordinates.
     */
    private Pose2d getShootPoseFor(Alliance alliance, AutoMode mode) {
        switch (mode) {
            case RED1:
                return new Pose2d(new Vector2d(RED1_SHOOT_X, RED1_SHOOT_Y), RED1_SHOOT_HE);
            case RED2:
                return new Pose2d(new Vector2d(RED2_SHOOT_X, RED2_SHOOT_Y), RED2_SHOOT_HE);
            case BLUE1:
                return new Pose2d(new Vector2d(BLUE1_SHOOT_X, BLUE1_SHOOT_Y), BLUE1_SHOOT_HE);
            case BLUE2:
                return new Pose2d(new Vector2d(BLUE2_SHOOT_X, BLUE2_SHOOT_Y), BLUE2_SHOOT_HE);
            case NONE:
            default:
                // Fallback: shouldn't ever be used in a real match.
                return new Pose2d(new Vector2d(0.0, 0.0), 0.0);
        }
    }

    /**
     * Stack poses per alliance/mode. Tune these for the real field.
     */
    private Pose2d[] getStackPosesFor(Alliance alliance, AutoMode mode) {
        switch (mode) {
            case RED1:
                return new Pose2d[]{
                        new Pose2d(new Vector2d(RED1_SPIKE1_X, RED1_SPIKE_Y), 0.0),
                        new Pose2d(new Vector2d(RED1_SPIKE2_X, RED1_SPIKE_Y), 0.0),
                        new Pose2d(new Vector2d(RED1_SPIKE3_X, RED1_SPIKE_Y), 0.0),
                };
            case RED2:
                return new Pose2d[]{
                        new Pose2d(new Vector2d(RED2_SPIKE1_X, RED2_SPIKE_Y), 0.0),
                        new Pose2d(new Vector2d(RED2_SPIKE2_X, RED2_SPIKE_Y), 0.0),
                        new Pose2d(new Vector2d(RED2_SPIKE3_X, RED2_SPIKE_Y), 0.0),
                };
            case BLUE1:
                return new Pose2d[]{
                        new Pose2d(new Vector2d(BLUE1_SPIKE1_X, BLUE1_SPIKE_Y), Math.PI),
                        new Pose2d(new Vector2d(BLUE1_SPIKE2_X, BLUE1_SPIKE_Y), Math.PI),
                        new Pose2d(new Vector2d(BLUE1_SPIKE3_X, BLUE1_SPIKE_Y), Math.PI),
                };
            case BLUE2:
                return new Pose2d[]{
                        new Pose2d(new Vector2d(BLUE2_SPIKE1_X, BLUE2_SPIKE_Y), Math.PI),
                        new Pose2d(new Vector2d(BLUE2_SPIKE2_X, BLUE2_SPIKE_Y), Math.PI),
                        new Pose2d(new Vector2d(BLUE2_SPIKE3_X, BLUE2_SPIKE_Y), Math.PI),
                };
            case NONE:
            default:
                return new Pose2d[0];
        }
    }

    @Override
    protected void initLoopExtended() {
        // Make sure subsystems exist for pre-start
        ensureSubsystems();

        // Let the indexer run its internal state machine.
        indexer.loop();

        // Optional: you can choose to keep intake stopped during preload and
        // just let humans drop into the hopper by hand.
        intake.stop();

        // Telemetry so drive team can see queue status during INIT
        telemetry.addData("Preload S0",  indexer.getS0());
        telemetry.addData("Preload S1L", indexer.getS1L());
        telemetry.addData("Preload S2",  indexer.getS2());

        runIndexerPreload();
    }

    private void ensureSubsystems() {
        if (intake == null) {
            intake = new IntakeSubsystem(hardwareMap);
        }
        if (indexer == null) {
            indexer = new IndexerSubsystem(hardwareMap);
        }
        if (shooter == null) {
            shooter = new ShooterSubsystem(hardwareMap);
        }
    }

    private boolean isQueueFull() {
        return indexer.getS0()  != IndexerSubsystem.Item.NONE &&
                indexer.getS1L() != IndexerSubsystem.Item.NONE &&
                indexer.getS2()  != IndexerSubsystem.Item.NONE;
    }

    /**
     * INIT-only auto-preload:
     * - Keep indexer SM updated.
     * - If not stepping and queue not full: detect color at S1L, and advance one step when we see a ball.
     * - If queue is full: still sample color for S1L but do not move.
     */
    private void runIndexerPreload() {
        // Keep state machine updated
        indexer.loop();
        boolean isStepping = indexer.isStepping();

        boolean queueFull = isQueueFull();
        lastStepDetectedColor = false;

        if (!isStepping) {
            if (!queueFull) {
                boolean colorPresent = indexer.detectAtS1L();
                if (colorPresent) {
                    lastStepDetectedColor = true;
                    lastDetectedItem = indexer.getS1L();
                    indexer.startStep(); // advance away from the ball
                }
            } else {
                // Queue full: keep S1L's color up-to-date, but don't move
                indexer.detectAtS1L();
            }
        }

        // Telemetry similar to Test_Intake_Indexer so you can debug easily
        telemetry.addData("queueFull", queueFull);
        telemetry.addData("lastDetectedItem", "%s (updatedThisLoop=%b)",
                lastDetectedItem, lastStepDetectedColor);
        telemetry.addData("colorMode",
                indexer.isRevColorSensorHealthy()
                        ? "REV"
                        : indexer.isDioColorAvailable()
                        ? "DIO (fallback)"
                        : "NO COLOR SENSOR");
    }

    /**
     * Wrap the ShooterSubsystem auton burst helper into a Road Runner Action.
     * Assumes you implemented:
     *   - shooter.startAutonBurst(int shots)
     *   - boolean shooter.updateAutonBurst(IndexerSubsystem indexer)
     * exactly like we just built for teleop.
     */
    private Action makeShootBurstAction(ShooterSubsystem shooter,
                                        IndexerSubsystem indexer,
                                        int shots,
                                        Alliance alliance,
                                        AutoMode autoMode,
                                        String label) {
        return new Action() {
            private boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!started) {
                    shooter.startAutonBurst(shots);
                    started = true;
                }

                // Run the shooter/indexer auton burst state machine
                boolean done = shooter.updateAutonBurst(indexer);
                shooter.loop();
                indexer.loop();

                // While we’re aimed at the goal and shooting,
                // optionally fuse goal tags into odometry.
                if (vision != null && Constants.Vision.ODO_FUSE_DURING_AUTO) {
                    boolean preferRedGoal = (alliance == Alliance.RED);
                    // pauseVision=false (we’re allowed to stream),
                    // allowMotifScan=false (motif already handled pre-start),
                    // allowOdoFuse=true (we WANT corrections here).
                    vision.loop(
                            false,
                            false,
                            true,
                            preferRedGoal
                    );
                }

                if (done) {
                    packet.put("AutoShoot",
                            String.format("%s %s (%s) done", alliance, autoMode, label));
                    return false; // action complete
                }
                return true; // keep running
            }
        };
    }
}
