package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AllianceDetector;
import org.firstinspires.ftc.teamcode.util.AutoSelector;
import org.firstinspires.ftc.teamcode.util.AutoSelector.AutoMode;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;

import org.firstinspires.ftc.teamcode.vision.AprilTagVisionManager;

import java.util.EnumMap;

/**
 * Base RR 1.0 autonomous for 2026DECODE.
 *
 * Responsibilities:
 *  - Read alliance + auto mode from digital inputs (AllianceDetector + AutoSelector).
 *  - Resolve a starting pose for the combo.
 *  - Construct MecanumDrive (PinpointLocalizer is wired inside).
 *  - Build and run an Action provided by subclasses.
 *
 * All units: inches and radians.
 */
public abstract class BaseAutoRR extends LinearOpMode {

    protected TelemetryUtil T;
    protected AllianceDetector allianceDetector;
    protected AutoSelector autoSelector;
    protected AprilTagVisionManager vision;

    private final EnumMap<AutoMode, Pose2d> startPoses = new EnumMap<>(AutoMode.class);

    public BaseAutoRR() {
        // Placeholder poses; replace with real field coordinates.
        startPoses.put(AutoMode.RED1,  new Pose2d(new Vector2d(0.0, 0.0), 0.0));
        startPoses.put(AutoMode.RED2,  new Pose2d(new Vector2d(0.0, 0.0), 0.0));
        startPoses.put(AutoMode.BLUE1, new Pose2d(new Vector2d(0.0, 0.0), 0.0));
        startPoses.put(AutoMode.BLUE2, new Pose2d(new Vector2d(0.0, 0.0), 0.0));
        startPoses.put(AutoMode.NONE,  new Pose2d(new Vector2d(0.0, 0.0), 0.0));
    }

    /**
     * Subclasses implement this to build the RR Action sequence.
     */
    protected abstract Action buildRoutine(MecanumDrive drive,
                                           Alliance alliance,
                                           AutoMode autoMode,
                                           Pose2d startPose);

    /**
     * Override if you want a different start-pose lookup.
     */
    protected Pose2d getStartPose(Alliance alliance, AutoMode mode) {
        Pose2d pose = startPoses.get(mode);
        if (pose == null) {
            return new Pose2d(new Vector2d(0.0, 0.0), 0.0);
        }
        return pose;
    }

    @Override
    public void runOpMode() {
        T = new TelemetryUtil(this);
        allianceDetector = new AllianceDetector(hardwareMap);
        autoSelector = new AutoSelector(hardwareMap);

        Alliance alliance = Alliance.NONE;
        AutoMode autoMode = AutoMode.NONE;

        vision = new AprilTagVisionManager(hardwareMap, T);

        // Pre-start loop: update switch state and display selection.
        while (!isStarted() && !isStopRequested()) {
            alliance = allianceDetector.determineAlliance();
            autoMode = autoSelector.select(alliance);

            T.banner(1, "RR Auto Init");
            T.t(1, "Alliance", alliance);
            T.t(1, "AutoMode", autoMode);
            T.t(2, "FlagA asserted", allianceDetector.isFlagAAsserted());
            T.t(2, "FlagB asserted", allianceDetector.isFlagBAsserted());
            T.t(2, "AutoA asserted", autoSelector.isAutoAAsserted());
            T.t(2, "AutoB asserted", autoSelector.isAutoBAsserted());
            telemetry.update();

            idle();
        }

        if (isStopRequested()) return;

        Pose2d startPose = getStartPose(alliance, autoMode);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Action routine = buildRoutine(drive, alliance, autoMode, startPose);

        if (routine != null) {
            Actions.runBlocking(routine);
        }
    }
}
