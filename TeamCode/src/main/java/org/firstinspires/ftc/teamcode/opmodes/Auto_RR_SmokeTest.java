package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoSelector.AutoMode;

/**
 * Simple RR 1.0 autonomous to validate:
 *  - MecanumDrive + PinpointLocalizer wiring
 *  - Alliance/auto switch reading via BaseAutoRR
 *
 * Path: forward then strafe left in field coordinates.
 */
@Config
@Autonomous(name = "Auto_RR_SmokeTest", group = "RR")
//@Disabled
public final class Auto_RR_SmokeTest extends BaseAutoRR {

    // Dashboard-tunable distances (inches).
    public static double FWD_DIST = 24.0;
    public static double STRAFE_DIST = 12.0;

    @Override
    protected Action buildRoutine(MecanumDrive drive,
                                  Alliance alliance,
                                  AutoMode autoMode,
                                  Pose2d startPose) {

        // Extremely simple path: forward, then left, from the starting pose.
        Action motion = drive.actionBuilder(startPose)
                .lineToX(startPose.position.x + FWD_DIST)
                .strafeTo(new Vector2d(
                        startPose.position.x + FWD_DIST,
                        startPose.position.y + STRAFE_DIST
                ))
                .build();

        // Wrap with a one-shot telemetry action for confirmation.
        return new SequentialAction(
                motion,
                new Action() {
                    private boolean done = false;

                    @Override
                    public boolean run(@NonNull TelemetryPacket packet) {
                        if (!done) {
                            telemetry.addData(
                                    "RR",
                                    "SmokeTest complete (alliance=%s, mode=%s)",
                                    alliance, autoMode
                            );
                            telemetry.update();
                            done = true;
                        }
                        return false; // one-shot
                    }
                }
        );
    }
}
