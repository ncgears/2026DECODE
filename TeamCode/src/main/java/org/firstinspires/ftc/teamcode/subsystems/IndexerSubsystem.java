package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
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
    private final ColorSensor colorSensor; //REV Color Sensor v3 when enabled
    private final boolean useRevColorSensor;
    private boolean revSensorHealthy = false;
    private boolean dioColorAvailable = false;

    private final EdgeDebounce slotDebounce;
    private final SimpleTimer stepTimer = new SimpleTimer();

    private boolean stepping = false;
    private boolean lastStepTimedOut = false;

    private int s0Index = 0; // 0:S0, 1:S1L, 2:S2
    private final Item[] q = new Item[] { Item.NONE, Item.NONE, Item.NONE };

    public IndexerSubsystem(HardwareMap hw) {
        useRevColorSensor = Constants.Indexer.USE_REV_COLOR_SENSOR;
        servo = hw.get(CRServo.class, Constants.Indexer.SERVO);
        slotSwitch = hw.get(DigitalChannel.class, Constants.Indexer.SLOT_LIMIT);
        slotSwitch.setMode(DigitalChannel.Mode.INPUT);
        slotDebounce = new EdgeDebounce(slotSwitch.getState(), Constants.Indexer.DEBOUNCE_MS);

        // --- Always try to wire DIO color inputs for fallback ---
        DigitalChannel purple = null;
        DigitalChannel green  = null;
        boolean dioOk = false;
        try {
            purple = hw.get(DigitalChannel.class, Constants.Indexer.PURPLE_DI);
            green  = hw.get(DigitalChannel.class, Constants.Indexer.GREEN_DI);
            purple.setMode(DigitalChannel.Mode.INPUT);
            green.setMode(DigitalChannel.Mode.INPUT);
            dioOk = true;
        } catch (Exception e) {
            // If wiring or config is missing, we just won't have DIO fallback
            dioOk = false;
        }
        diPurple = purple;
        diGreen  = green;
        dioColorAvailable = dioOk;

        // --- Try to wire REV color sensor (optional) ---
        ColorSensor cs = null;
        boolean revOk = false;
        if (useRevColorSensor) {
            try {
                cs = hw.get(ColorSensor.class, Constants.Indexer.COLOR_SENSOR);

                // Quick sanity check: real sensor should not read all zeros.
                // Do a few samples to avoid a one-off weird read during init.
                boolean sane = false;
                for (int i = 0; i < 3; i++) {
                    int r = cs.red();
                    int g = cs.green();
                    int b = cs.blue();
                    int a = cs.alpha();

                    if (r != 0 || g != 0 || b != 0 || a != 0) {
                        sane = true;
                        break;
                    }
                }

                if (sane) {
                    revOk = true;
                } else {
                    // Treat as if hw.get() had failed: don't use this sensor
                    cs = null;
                    revOk = false;
                }
            } catch (Exception e) {
                cs = null;
                revOk = false;
            }
        }
        colorSensor = cs;
        revSensorHealthy = revOk;
    }

    /**
     * Start a step where the S0 piece is being fed into the shooter.
     *
     * Semantics:
     *  - The piece currently at S0 is considered GONE (shot) and is cleared
     *    from the logical queue before we rotate.
     *  - The step timing / jam detection is identical to startStep().
     *
     * Use this ONLY when the shooter ramp is up and we are intentionally
     * feeding a game piece into the shooter.
     */
    public void startStepForShot() {
        // Clear the artifact at S0; by the time the step completes,
        // that physical piece has been launched and no longer exists.
        q[s0Index] = Item.NONE;

        // Then do a normal step.
        startStep();
    }

    /** Begin stepping forward one slot (S1L->S0->S2->S1L). */
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
    public Item getS1L() { return q[mod(s0Index + 2, 3)]; }
    public Item getS2()  { return q[mod(s0Index + 1, 3)]; }
    public void setS1L(Item it) { q[mod(s0Index + 2, 3)] = it; }
    public void setS0(Item it)  { q[s0Index] = it; }
    public void setS2(Item it)  { q[mod(s0Index + 1, 3)] = it; }

    public boolean isQueueFull() { return getS0()!=Item.NONE && getS1L()!=Item.NONE && getS2()!=Item.NONE; }

    private Item readColorFromDio() {
        if (!dioColorAvailable || diPurple == null || diGreen == null) {
            return Item.NONE;
        }

        // ACTIVE-LOW lines
        boolean purpleActive = !diPurple.getState();
        boolean greenActive  = !diGreen.getState();

        if (purpleActive ^ greenActive) {
            return purpleActive ? Item.PURPLE : Item.GREEN;
        }
        return Item.NONE;
    }

    private Item readColorFromRevUnsafe() {
        // Preconditions: colorSensor != null and revSensorHealthy assumed.
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();
        int a = colorSensor.alpha();

        if (a < Constants.Indexer.COLOR_ALPHA_MIN) {
            return Item.NONE;
        }

        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);
        float hue = hsv[0];
        float sat = hsv[1];

        if (sat < Constants.Indexer.COLOR_MIN_SAT) {
            return Item.NONE;
        }

        if (hue >= Constants.Indexer.PURPLE_HUE_MIN && hue <= Constants.Indexer.PURPLE_HUE_MAX) {
            return Item.PURPLE;
        }
        if (hue >= Constants.Indexer.GREEN_HUE_MIN && hue <= Constants.Indexer.GREEN_HUE_MAX) {
            return Item.GREEN;
        }

        return Item.NONE;
    }

    private Item readColorAtS1LRaw() {
        // 1) Preferred path: REV Color Sensor (if enabled AND still healthy)
        if (useRevColorSensor && colorSensor != null && revSensorHealthy) {
            try {
                return readColorFromRevUnsafe();
            } catch (Exception e) {
                // Something bad happened on I2C; permanently abandon REV
                revSensorHealthy = false;
                // Fall through to DIO fallback below
            }
        }

        // 2) Fallback path: DIO color inputs
        return readColorFromDio();
    }

    /**
     * Sample the color at S1L and (optionally) update the queue.
     *
     * Behavior:
     *   - If we classify a confident color (PURPLE or GREEN), S1L is set to that
     *     value (overwriting any previous value) and this returns true.
     *   - Otherwise, queue is unchanged and this returns false.
     *
     * Note: this NEVER sets a slot to NONE; clearing is only done by clearAll()/re-index.
     */
    public boolean detectAtS1L() {
        Item newItem = readColorAtS1LRaw();
        if (newItem != Item.NONE) {
            if (getS1L() != newItem) {
                setS1L(newItem);
            }
            return true;
        }
        // No clean detection this sample
        return false;
    }

    /**
     * Peek at the raw color at S1L without modifying the queue.
     *
     * Returns:
     *   - Item.PURPLE if we classify as purple
     *   - Item.GREEN  if we classify as green
     *   - Item.NONE   if no clean read / no piece
     *
     * This DOES NOT change S0/S1L/S2; it is purely diagnostic.
     */
    public Item peekColorAtS1LRaw() {
        return readColorAtS1LRaw();
    }

    // New helper: compute how many forward steps we need; do NOT touch s0Index.
    public int computeStepsForMotif(String motifCode) {
        if (!isQueueFull()) return 0;

        int idx = s0Index;
        int bestSteps = 0;

        // Try direct motif match
        for (int steps = 0; steps < 3; steps++) {
            String seq =
                    (q[idx]               == Item.GREEN ? "G" : "P") +
                            (q[mod(idx + 2, 3)]   == Item.GREEN ? "G" : "P") +
                            (q[mod(idx + 1, 3)]   == Item.GREEN ? "G" : "P");
            if (seq.equals(motifCode)) {
                bestSteps = steps;
                break;
            }
            idx = mod(idx + 2, 3); // simulate a step
        }

        // TODO: add your GGP special-case logic here using the simulated idx

        return bestSteps;
    }

    /**
     * Perform one physical forward step of the carousel and block until
     * the step completes (limit switch edge or timeout).
     *
     * Uses the existing startStep()/loop() machinery so s0Index and
     * timeout behavior remain consistent.
     */
    private void doPhysicalStepBlocking() {
        // If a step is already in progress, let it finish first
        if (!stepping) {
            startStep();
        }

        // Run the step state machine until it finishes.
        // loop() will:
        //  - stop the servo on debounced HIGH edge
        //  - update s0Index
        //  - or stop on timeout and set lastStepTimedOut
        while (stepping) {
            loop();
        }

        // We don't care about the timeout latched state for motif rotation,
        // so clear it to avoid surprising callers later.
        lastStepTimedOut = false;
    }

    /**
     * Physically rotate the carousel to match the requested motif, using only
     * forward steps. This ALWAYS moves the servo; there is no "logical-only"
     * rotation here.
     *
     * Behavior:
     *   1) If queue is not full, do nothing.
     *   2) Try up to 3 forward steps to match G/P pattern exactly.
     *   3) If we have GGP loaded and motif is PGP/PPG, ensure the *next*
     *      shot (S0) is PURPLE by stepping until PURPLE is at S0.
     */
    public void rotateForMotif(String motifCode) {
        if (!isQueueFull()) return;

        // Only handle the three known motif codes; ignore anything else.
        if (!"GPP".equals(motifCode) &&
                !"PGP".equals(motifCode) &&
                !"PPG".equals(motifCode)) {
            return;
        }

        // 1) Direct motif match by up to three forward steps
        for (int tries = 0; tries < 3; tries++) {
            String seq =
                    (getS0()  == Item.GREEN ? "G" : "P") +
                            (getS1L() == Item.GREEN ? "G" : "P") +
                            (getS2()  == Item.GREEN ? "G" : "P");

            if (seq.equals(motifCode)) {
                // Already aligned; no more motion needed.
                return;
            }

            // Physically advance one slot forward (updates s0Index via loop()).
            doPhysicalStepBlocking();
        }

        // 2) Special case: GGP loaded, motif PGP/PPG => make S0 purple
        int greens =
                (getS0()  == Item.GREEN ? 1 : 0) +
                        (getS1L() == Item.GREEN ? 1 : 0) +
                        (getS2()  == Item.GREEN ? 1 : 0);
        int purples = 3 - greens;

        if (greens == 2 &&
                purples == 1 &&
                ("PGP".equals(motifCode) || "PPG".equals(motifCode))) {

            // Step forward until PURPLE lands at S0, or we've tried all 3 slots.
            for (int i = 0; i < 3; i++) {
                if (getS0() == Item.PURPLE) {
                    break;
                }
                doPhysicalStepBlocking();
            }
        }
    }

    public void clearAll() { q[0]=Item.NONE; q[1]=Item.NONE; q[2]=Item.NONE; }

    public boolean isRevColorSensorHealthy() { return revSensorHealthy; }
    public boolean isDioColorAvailable()     { return dioColorAvailable; }

    public void startUnjamReverse() {
        if (stepping) return;
        stepping = true;
        stepTimer.reset();
        lastStepTimedOut = false;
        double p = Constants.Indexer.POWER_FWD * (Constants.Indexer.DIR_FORWARD_IS_POSITIVE ? -1.0 : +1.0);
        servo.setPower(p);
    }

    public void stopUnjam() {
        stopServo();
        stepping = false;
    }

    private static int mod(int a,int b){ int m=a%b; return m<0?m+b:m; }
}
