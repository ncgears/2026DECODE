package org.firstinspires.ftc.teamcode.util;

/** Simple stateful debounce for digital inputs; returns true when an edge is accepted. */
public class EdgeDebounce {
    private boolean last;
    private long lastChangeMs;
    private final int debounceMs;

    public EdgeDebounce(boolean initial, int debounceMs) {
        this.last = initial;
        this.debounceMs = debounceMs;
        this.lastChangeMs = System.currentTimeMillis();
    }

    /** @return true if a (debounced) edge changed the stored state. */
    public boolean update(boolean current) {
        long now = System.currentTimeMillis();
        if (current != last && (now - lastChangeMs) >= debounceMs) {
            last = current;
            lastChangeMs = now;
            return true;
        }
        return false;
    }
    public boolean get() { return last; }
}
