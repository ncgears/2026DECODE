package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.constants.Constants.Controls.*;

public class Shaping {
    public static double shapeTranslate(double x) { return shape(x, EXPO_TRANSLATION); }
    public static double shapeRotate(double x) { return shape(x, EXPO_ROTATION); }

    private static double shape(double x, double expo) {
        switch (SHAPING) {
            case LINEAR: return applyDeadband(x, DEAD_BAND);
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
