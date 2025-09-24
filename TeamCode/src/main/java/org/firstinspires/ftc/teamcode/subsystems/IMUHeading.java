package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** IMU (BNO055) heading provider with zero-able yaw offset. */
public class IMUHeading implements HeadingProvider {
    private final IMU imu;
    private double yawOffset = 0.0;
    public IMUHeading(HardwareMap hw) {
        imu = hw.get(IMU.class, Constants.IMUCfg.NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                Constants.IMUCfg.LOGO, Constants.IMUCfg.USB)));
    }
    public double getHeadingRad() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return wrap(yaw - yawOffset);
    }
    public void zeroHeading() { yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); }
    private static double wrap(double a) { while (a > Math.PI) a -= 2*Math.PI; while (a < -Math.PI) a += 2*Math.PI; return a; }
}
