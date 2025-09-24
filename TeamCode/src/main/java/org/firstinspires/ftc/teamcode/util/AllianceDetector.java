package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.Constants;

/** Reads two digital inputs and returns inferred alliance color. */
public class AllianceDetector {
    private final DigitalChannel a, b;
    public AllianceDetector(HardwareMap hw) {
        a = hw.get(DigitalChannel.class, Constants.Digital.FLAG_A);
        b = hw.get(DigitalChannel.class, Constants.Digital.FLAG_B);
        a.setMode(DigitalChannel.Mode.INPUT);
        b.setMode(DigitalChannel.Mode.INPUT);
    }
    public Alliance determineAlliance() {
        try {
            boolean sa = a.getState(); // pull-ups: high normally
            boolean sb = b.getState();
            if (sa && sb) return Alliance.RED;    // no switches -> red flag
            if (sa || sb) return Alliance.BLUE;   // one switch -> blue flag
            return Alliance.NONE;                 // both low -> none/fault
        } catch (Exception e) {
            return Alliance.NONE;
        }
    }
}
