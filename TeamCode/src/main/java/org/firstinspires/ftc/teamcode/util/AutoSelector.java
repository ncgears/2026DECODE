package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.Constants;

/**
 * AutoSelector chooses among five modes based on alliance and two digital inputs:
 *   RED1, RED2, BLUE1, BLUE2, NONE.
 *
 * Assumptions:
 * - Inputs have pull-ups; HIGH when open, LOW when asserted.
 * - Exactly one of AUTO_A or AUTO_B should be asserted. Otherwise returns NONE.
 *
 * Mapping:
 * - Alliance RED:  AUTO_A -> RED1,  AUTO_B -> RED2
 * - Alliance BLUE: AUTO_A -> BLUE2, AUTO_B -> BLUE1 (reversed for blue)
 */
public class AutoSelector {
    public enum AutoMode { RED1, RED2, BLUE1, BLUE2, NONE }

    private final DigitalChannel autoA, autoB;

    public AutoSelector(HardwareMap hw) {
        autoA = hw.get(DigitalChannel.class, Constants.Digital.AUTO_A);
        autoB = hw.get(DigitalChannel.class, Constants.Digital.AUTO_B);
        autoA.setMode(DigitalChannel.Mode.INPUT);
        autoB.setMode(DigitalChannel.Mode.INPUT);
    }

    public AutoMode getAutoMode(Alliance alliance) { return select(alliance); } //convenience wrapper for logical name
    public AutoMode select(Alliance alliance) {
        boolean a = !autoA.getState(); // LOW => asserted
        boolean b = !autoB.getState(); // LOW => asserted

        if (a && b)   return AutoMode.NONE; // invalid
        if (!a && !b) return AutoMode.NONE; // not set

        switch (alliance) {
            case RED:
                return a ? AutoMode.RED1  : AutoMode.RED2;
            case BLUE:
                return a ? AutoMode.BLUE2 : AutoMode.BLUE1; // reversed
            default:
                return AutoMode.NONE;
        }
    }

    public boolean isAutoAAsserted() { return !autoA.getState(); }
    public boolean isAutoBAsserted() { return !autoB.getState(); }
}
