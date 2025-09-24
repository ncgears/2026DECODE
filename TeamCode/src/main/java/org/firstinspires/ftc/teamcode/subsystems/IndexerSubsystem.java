package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.util.EdgeDebounce;
import org.firstinspires.ftc.teamcode.util.SimpleTimer;

/** 3-slot carousel indexer with S1L color detection and slot limit switch. */
public class IndexerSubsystem {
    public enum Item { NONE, PURPLE, GREEN }

    private final CRServo servo;
    private final DigitalChannel slotSwitch;     // NC: HIGH normal, LOW pressed
    private final DigitalChannel diPurple, diGreen; // PNP: HIGH when detected

    private final EdgeDebounce slotDebounce;
    private final SimpleTimer stepTimer = new SimpleTimer();

    private boolean stepping = false;
    private boolean lastStepTimedOut = false;

    private int s0Index = 0; // 0:S0, 1:S1L, 2:S2
    private final Item[] q = new Item[] { Item.NONE, Item.NONE, Item.NONE };

    public IndexerSubsystem(HardwareMap hw) {
        servo = hw.get(CRServo.class, Constants.Indexer.SERVO);
        slotSwitch = hw.get(DigitalChannel.class, Constants.Indexer.SLOT_LIMIT);
        diPurple   = hw.get(DigitalChannel.class, Constants.Indexer.PURPLE_DI);
        diGreen    = hw.get(DigitalChannel.class, Constants.Indexer.GREEN_DI);
        slotSwitch.setMode(DigitalChannel.Mode.INPUT);
        diPurple.setMode(DigitalChannel.Mode.INPUT);
        diGreen.setMode(DigitalChannel.Mode.INPUT);
        slotDebounce = new EdgeDebounce(slotSwitch.getState(), Constants.Indexer.DEBOUNCE_MS);
    }

    /** Begin stepping forward one slot (S0->S2->S1L->S0). */
    public void startStep() {
        if (stepping) return;
        stepping = true;
        stepTimer.reset();
        lastStepTimedOut = false;
        double p = Constants.Indexer.POWER_FWD * (Constants.Indexer.DIR_FORWARD_IS_POSITIVE ? +1.0 : -1.0);
        servo.setPower(p);
    }

    /** Call each loop; stops on debounced HIGH edge or timeout. */
    public void loop() {
        boolean current = slotSwitch.getState(); // HIGH normal (NC), LOW pressed => wait for HIGH edge
        boolean changed = slotDebounce.update(current);
        if (stepping) {
            if (changed && slotDebounce.get()) {
                stopServo(); stepping = false;
                s0Index = mod(s0Index + 2, 3); // rotate forward one slot
            } else if (stepTimer.ms() > Constants.Indexer.STEP_TIMEOUT_MS) {
                stopServo(); stepping = false; lastStepTimedOut = true;
            }
        }
    }

    public void stopServo() { servo.setPower(0.0); }
    public boolean isStepping() { return stepping; }
    public boolean getAndClearTimedOut() { boolean v = lastStepTimedOut; lastStepTimedOut = false; return v; }

    public Item getS0()  { return q[s0Index]; }
    public Item getS1L() { return q[mod(s0Index + 1, 3)]; }
    public Item getS2()  { return q[mod(s0Index + 2, 3)]; }
    public void setS1L(Item it) { q[mod(s0Index + 1, 3)] = it; }
    public void setS0(Item it)  { q[s0Index] = it; }
    public void setS2(Item it)  { q[mod(s0Index + 2, 3)] = it; }

    public boolean isQueueFull() { return getS0()!=Item.NONE && getS1L()!=Item.NONE && getS2()!=Item.NONE; }

    /** Read color lines at S1L; overwrite queue if exactly one line is HIGH. */
    public boolean detectAtS1L() {
        boolean p = diPurple.getState(), g = diGreen.getState();
        if (p ^ g) { setS1L(p ? Item.PURPLE : Item.GREEN); return true; }
        return false;
    }

    /** Rotate buffer to match motif order, with special GGP rule for PGP/PPG => make next shot PURPLE. */
    public void rotateForMotif(String motifCode) {
        if (!isQueueFull()) return;
        // Try to match motif directly by rotating up to 3 times
        for (int tries=0; tries<3; tries++) {
            String seq = (getS0()==Item.GREEN?"G":"P") + (getS1L()==Item.GREEN?"G":"P") + (getS2()==Item.GREEN?"G":"P");
            if (seq.equals(motifCode)) return;
            s0Index = mod(s0Index + 2, 3);
        }
        // Special case: GGP set + motif PGP/PPG -> next should be Purple
        int greens = (getS0()==Item.GREEN?1:0) + (getS1L()==Item.GREEN?1:0) + (getS2()==Item.GREEN?1:0);
        int purples = 3 - greens;
        if (greens==2 && purples==1 && ("PGP".equals(motifCode) || "PPG".equals(motifCode))) {
            for (int i=0;i<3;i++) { if (getS0()==Item.PURPLE) break; s0Index = mod(s0Index + 2, 3); }
        }
    }

    public void clearAll() { q[0]=Item.NONE; q[1]=Item.NONE; q[2]=Item.NONE; }

    private static int mod(int a,int b){ int m=a%b; return m<0?m+b:m; }
}
