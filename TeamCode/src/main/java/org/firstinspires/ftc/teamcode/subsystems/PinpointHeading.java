package org.firstinspires.ftc.teamcode.subsystems;

/** Placeholder until goBILDA Pinpoint SDK is integrated. */
public class PinpointHeading implements HeadingProvider {
    private double offsetRad = 0.0;
    public PinpointHeading(com.qualcomm.robotcore.hardware.HardwareMap hw) {}
    public double getHeadingRad() { return 0.0 - offsetRad; }
    public void zeroHeading() { offsetRad = getHeadingRad(); }
}
