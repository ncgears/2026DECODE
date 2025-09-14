package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

/** Placeholder that compiles without the goBILDA driver. Replace with real Pinpoint reads. */
public class PinpointHeading implements HeadingProvider {
    private double offsetRad = 0.0;

    public PinpointHeading(com.qualcomm.robotcore.hardware.HardwareMap hw) {
        Log.i("PinpointHeading", "Placeholder initialized. Replace with goBILDA Pinpoint driver calls.");
    }

    @Override public double getHeadingRad() {
        double heading = 0.0; // TODO: return fused heading from Pinpoint in radians
        return heading - offsetRad;
    }

    @Override public void zeroHeading() { offsetRad = getHeadingRad(); }
}
