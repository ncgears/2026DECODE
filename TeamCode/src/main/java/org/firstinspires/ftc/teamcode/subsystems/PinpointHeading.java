package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.Constants;

/**
 * PinpointHeading
 *
 * HeadingProvider implementation backed by the goBILDA Pinpoint Odometry Computer.
 *
 * Conventions:
 *  - Robot frame: +X forward, +Y left.
 *  - getHeadingRad() returns yaw in radians, wrapped to [-pi, pi].
 *  - zeroHeading() defines the current yaw as 0.
 *
 * Geometry:
 *  The asymmetric pod locations are defined in Constants.Pinpoint as:
 *   - X_POD_SIDE_OFFSET_MM:  sideways position (mm) of the X (forward) pod, left-positive.
 *   - Y_POD_FORWARD_OFFSET_MM: forward position (mm) of the Y (strafe) pod, forward-positive.
 */
public class PinpointHeading implements HeadingProvider {

    private final GoBildaPinpointDriver pinpoint;
    private double yawOffsetRad = 0.0;

    public PinpointHeading(HardwareMap hw) {
        // Get the built-in goBILDA Pinpoint device by type + name.
        // Make sure Constants.Pinpoint.NAME matches the RC config device name ("pinpoint").
        pinpoint = hw.get(GoBildaPinpointDriver.class, Constants.Pinpoint.NAME);

        // Use factory ticks-per-mm for goBILDA 4-Bar odometry pods
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // Set asymmetric pod offsets (robot-centric, in mm):
        //  - xOffset: sideways distance from robot center (left-positive) for X pod
        //  - yOffset: forward distance from robot center (forward-positive) for Y pod
        pinpoint.setOffsets(
                Constants.Pinpoint.X_POD_SIDE_OFFSET_MM,
                Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM,
                DistanceUnit.MM
        );

        // Default encoder directions: both FORWARD.
        // Flip signs here if X decreases when driving forward or Y decreases when strafing left.
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset pose and IMU bias at startup. Robot must be stationary.
        pinpoint.resetPosAndIMU();
    }

    @Override
    public double getHeadingRad() {
        // Fast path: update only heading data from the Pinpoint.
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        // Built-in driver returns heading in the requested AngleUnit.
        double yawRad = pinpoint.getHeading(AngleUnit.RADIANS);
        return wrap(yawRad - yawOffsetRad);
    }

    @Override
    public void zeroHeading() {
        // Capture the current yaw as the new zero.
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        yawOffsetRad = pinpoint.getHeading(AngleUnit.RADIANS);
    }

    private static double wrap(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
