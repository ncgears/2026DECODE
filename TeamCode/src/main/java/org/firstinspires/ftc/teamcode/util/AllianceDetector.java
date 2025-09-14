package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.Constants;

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
            boolean sa = a.getState();
            boolean sb = b.getState();
            if (sa && sb) return Alliance.RED;
            if (sa || sb) return Alliance.BLUE;
            return Alliance.NONE;
        } catch (Exception e) {
            return Alliance.NONE;
        }
    }
}
