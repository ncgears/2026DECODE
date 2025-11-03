package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.Constants;

/**
 * Alliance detector using two digital inputs:
 * - FLAG_A => Red (default)
 * - FLAG_B => Blue (default)
 * Set Constants.Digital.SWAP_FLAG_SW = true to swap the mapping (A->Blue, B->Red).
 *
 * Notes:
 * - REV hubs use pull-ups; inputs read HIGH when open, LOW when asserted.
 * - If both or neither are asserted, returns NONE.
 */
public class AllianceDetector {
    private final DigitalChannel flagA, flagB;

    public AllianceDetector(HardwareMap hw) {
        flagA = hw.get(DigitalChannel.class, Constants.Digital.FLAG_A);
        flagB = hw.get(DigitalChannel.class, Constants.Digital.FLAG_B);
        flagA.setMode(DigitalChannel.Mode.INPUT);
        flagB.setMode(DigitalChannel.Mode.INPUT);
    }

    public Alliance determineAlliance() {
        boolean aTrig = !flagA.getState(); // LOW => asserted
        boolean bTrig = !flagB.getState(); // LOW => asserted

        if (aTrig && bTrig) return Alliance.NONE; // invalid / both switches
        if (!aTrig && !bTrig) return Alliance.NONE; // none

        boolean aMeansRed = !Constants.Digital.SWAP_FLAG_SW; // default: A->RED
        if (aTrig)  return aMeansRed ? Alliance.RED  : Alliance.BLUE;
        else        return aMeansRed ? Alliance.BLUE : Alliance.RED;
    }

    /** Raw asserted states for debugging (true == asserted/LOW). */
    public boolean isFlagAAsserted() { return !flagA.getState(); }
    public boolean isFlagBAsserted() { return !flagB.getState(); }
}
