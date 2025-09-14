package org.firstinspires.ftc.teamcode.util;

public class SlewRateLimiter {
    private final double ratePerSec;
    private double last;
    private long lastTimeNs;

    public SlewRateLimiter(double ratePerSec) {
        this.ratePerSec = Math.max(1e-9, ratePerSec);
        this.lastTimeNs = System.nanoTime();
    }

    public double calculate(double input) {
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        double maxDelta = ratePerSec * dt;
        double delta = input - last;
        if (delta > maxDelta) delta = maxDelta;
        else if (delta < -maxDelta) delta = -maxDelta;
        last += delta;
        return last;
    }

    public void reset(double value) { last = value; lastTimeNs = System.nanoTime(); }
}
