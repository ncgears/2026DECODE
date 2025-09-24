package org.firstinspires.ftc.teamcode.util;

/** Millisecond timer helper. */
public class SimpleTimer {
    private long startMs;
    public SimpleTimer() { reset(); }
    public void reset() { startMs = System.currentTimeMillis(); }
    public long ms() { return System.currentTimeMillis() - startMs; }
}
