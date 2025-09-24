package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.constants.Constants;

/** Telemetry with configurable verbosity and simple banners. */
public class TelemetryUtil {
    private final OpMode op;
    public TelemetryUtil(OpMode op) { this.op = op; }
    public void t(int lvl, String caption, Object value) {
        if (lvl <= Constants.TelemetryCfg.TELEMETRY_LEVEL) op.telemetry.addData(caption, value);
    }
    public void tl(int lvl, String line) {
        if (lvl <= Constants.TelemetryCfg.TELEMETRY_LEVEL) op.telemetry.addLine(line);
    }
    public void banner(int lvl, String msg) {
        if (lvl <= Constants.TelemetryCfg.TELEMETRY_LEVEL) op.telemetry.addLine("==== " + msg + " ====");
    }
}
