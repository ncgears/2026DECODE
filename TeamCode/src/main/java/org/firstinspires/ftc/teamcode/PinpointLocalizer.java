package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.Constants;

/**
 * PinpointLocalizer
 *
 * Road Runner 1.0 Localizer implementation backed by the goBILDA Pinpoint.
 *
 * Conventions:
 *  - Robot frame: +X forward, +Y left (matches Constants + PinpointHeading).
 *  - Pose reported to Road Runner is in **inches**, heading in radians.
 *
 * Geometry:
 *  The asymmetric pod locations are defined in Constants.Pinpoint. We assume that
 *  the Pinpoint itself has already been configured with these offsets and any
 *  necessary calibration (JSON file, etc).
 *
 *  That means Pinpoint's reported (x, y, heading) are already the robot-frame pose
 *  we want, so this class is mostly unit conversion + RR interface glue.
 */
public final class PinpointLocalizer implements Localizer {

    private static final double MM_PER_IN = 25.4;

    private final GoBildaPinpointDriver pinpoint;

    // Road Runner pose in **inches**.
    private Pose2d poseInches = new Pose2d(0.0, 0.0, 0.0);

    // Last pose sample (for finite-difference velocity).
    private Pose2d lastPoseInches = new Pose2d(0.0, 0.0, 0.0);
    private long lastSampleNanos = 0L;

    public PinpointLocalizer(@NonNull HardwareMap hardwareMap, @NonNull Pose2d initialPose) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.Pinpoint.NAME);

        // Basic Pinpoint configuration is assumed to be done elsewhere (your PinpointHeading
        // subsystem and/or initialization opmode). Here we only make sure units are as expected.
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.EncoderResolution.ENCODER_8192_TICKS);
//        pinpoint.setDistanceUnit(DistanceUnit.MM);
//        pinpoint.setAngularUnit(AngleUnit.RADIANS);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        setPose(initialPose);
        lastPoseInches = poseInches;
        lastSampleNanos = System.nanoTime();
    }

    public GoBildaPinpointDriver getDriver() {
        return pinpoint;
    }

    @Override
    public void setPose(Pose2d pose) {
        // Store the RR pose in inches and also push a matching pose into Pinpoint
        // (in mm) so both agree. This assumes Pinpoint's "setPosition" semantics
        // treat (x, y, heading) as absolute in robot frame.
        poseInches = pose;

        double xMm = pose.position.x * MM_PER_IN;
        double yMm = pose.position.y * MM_PER_IN;
        double headingRad = pose.heading.toDouble();

        // Build an FTC Pose2D in mm/radians
        Pose2D pinPose = new Pose2D(
                DistanceUnit.MM,
                xMm,
                yMm,
                AngleUnit.RADIANS,
                headingRad
        );

        // Push pose into Pinpoint so its internal frame matches RR's
        pinpoint.setPosition(pinPose);
    }

    @Override
    public Pose2d getPose() {
        return poseInches;
    }

    @Override
    public PoseVelocity2d update() {
        // Query Pinpoint for latest pose.
        pinpoint.update();

        double xMm = pinpoint.getPosX(DistanceUnit.MM);
        double yMm = pinpoint.getPosY(DistanceUnit.MM);
        double headingRad = pinpoint.getHeading(AngleUnit.RADIANS);

        Pose2d newPoseInches = new Pose2d(
                new Vector2d(xMm / MM_PER_IN, yMm / MM_PER_IN),
                headingRad
        );

        long now = System.nanoTime();
        double dt = lastSampleNanos == 0L
                ? 0.0
                : (now - lastSampleNanos) * 1e-9;

        Vector2d vel = new Vector2d(0.0, 0.0);
        double angVel = 0.0;

        if (dt > 1e-4) {
            double vx = (newPoseInches.position.x - lastPoseInches.position.x) / dt;
            double vy = (newPoseInches.position.y - lastPoseInches.position.y) / dt;
            double dTheta = wrapAngle(newPoseInches.heading.toDouble() - lastPoseInches.heading.toDouble());
            double w = dTheta / dt;

            vel = new Vector2d(vx, vy);
            angVel = w;
        }

        lastPoseInches = newPoseInches;
        lastSampleNanos = now;
        poseInches = newPoseInches;

        return new PoseVelocity2d(vel, angVel);
    }

    private static double wrapAngle(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
