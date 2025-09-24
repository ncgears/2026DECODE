package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.constants.Constants.Controls.*;

/** Deadband + expo (or linear/squared) shaping for sticks, with legacy aliases. */
public class Shaping {
    // Preferred names
    public static double translate(double x) { return shape(x, EXPO_TRANSLATION); }
    public static double rotate(double x) { return shape(x, EXPO_ROTATION); }

    // Legacy compatibility (older TeleOp_Drive code)
    public static double shapeTranslate(double x) { return translate(x); }
    public static double shapeRotate(double x) { return rotate(x); }

    private static double shape(double x, double expo) {
        switch (SHAPING) {
            case LINEAR:  return applyDeadband(x, DEAD_BAND);
            case SQUARED: return squared(applyDeadband(x, DEAD_BAND));
            case DEAD_EXPO:
            default:
                double v = applyDeadband(x, DEAD_BAND);
                return (1 - expo) * v + expo * v * v * v;
        }
    }
    private static double applyDeadband(double x, double db) {
        if (Math.abs(x) < db) return 0.0;
        return Math.copySign((Math.abs(x) - db) / (1.0 - db), x);
    }
    private static double squared(double x) { return Math.copySign(x * x, x); }
}
