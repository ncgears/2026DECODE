package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.external.pinpoint.GoBildaPinpointDriver;

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
 *
 * This class only concerns itself with heading; if you later want full pose, build a
 * separate PinpointOdometry wrapper that exposes getPose()/getVelocity().
 */
public class PinpointHeading implements HeadingProvider {

    private final GoBildaPinpointDriver pinpoint;
    private double yawOffsetRad = 0.0;

    public PinpointHeading(HardwareMap hw) {
        // I2C device name for the Pinpoint comes from RC config via Constants.Pinpoint.NAME
        I2cDeviceSynchSimple i2c =
                hw.get(I2cDeviceSynchSimple.class, Constants.Pinpoint.NAME);

        // true => this driver owns the I2C client instance
        pinpoint = new GoBildaPinpointDriver(i2c, true);

        // Initialize underlying hardware
        pinpoint.initialize();

        // Use factory ticks-per-mm for goBILDA 4-Bar pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set asymmetric pod offsets (robot-centric, in mm):
        //  - X pod offset: sideways distance from robot center (left-positive)
        //  - Y pod offset: forward distance from robot center (forward-positive)
        pinpoint.setOffsets(
                Constants.Pinpoint.X_POD_SIDE_OFFSET_MM,
                Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM
        );

        // Default encoder directions: both FORWARD.
        // If you find X decreases when driving forward or Y decreases when strafing left,
        // flip the corresponding direction here.
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
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);

        double yawDeg = pinpoint.getHeading();
        double yawRad = Math.toRadians(yawDeg);
        return wrap(yawRad - yawOffsetRad);
    }

    @Override
    public void zeroHeading() {
        // Capture the current yaw as the new zero
        pinpoint.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        double yawDeg = pinpoint.getHeading();
        yawOffsetRad = Math.toRadians(yawDeg);
    }

    private static double wrap(double angle) {
        while (angle > Math.PI)  angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
