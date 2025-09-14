package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.util.TelemetryUtil;
import org.json.JSONObject;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

@TeleOp(name = "Calibration_Pinpoint", group = "Tuning")
public class Calibration_Pinpoint extends OpMode {
    private TelemetryUtil T;
    private double trackWidth = Constants.Pinpoint.TRACK_WIDTH_MM;
    private double fwdOffset = Constants.Pinpoint.FORWARD_OFFSET_MM;

    @Override public void init() { T = new TelemetryUtil(this); loadJson(); }

    @Override public void loop() {
        double step = gamepad1.left_bumper ? 0.5 : (gamepad1.right_bumper ? 5.0 : 1.0);
        if (gamepad1.dpad_left) trackWidth -= step;
        if (gamepad1.dpad_right) trackWidth += step;
        if (gamepad1.dpad_down) fwdOffset -= step;
        if (gamepad1.dpad_up) fwdOffset += step;

        if (gamepad1.a) saveJson();
        if (gamepad1.b) loadJson();
        if (gamepad1.x) { trackWidth = 150.0; fwdOffset = 0.0; }

        T.t(4, "TrackWidth(mm)", trackWidth);
        T.t(4, "FwdOffset(mm)", fwdOffset);
        T.t(2, "A=Save  B=Load  X=Reset  LB=Fine  RB=Coarse", "");
        telemetry.update();
    }

    private void saveJson() {
        try {
            File f = new File(Constants.Pinpoint.CALIBRATION_JSON);
            f.getParentFile().mkdirs();
            JSONObject o = new JSONObject();
            o.put("trackWidthMm", trackWidth);
            o.put("forwardOffsetMm", fwdOffset);
            try (FileWriter fw = new FileWriter(f)) { fw.write(o.toString()); }
            Constants.Pinpoint.TRACK_WIDTH_MM = trackWidth;
            Constants.Pinpoint.FORWARD_OFFSET_MM = fwdOffset;
        } catch (Exception e) {
            telemetry.addData("ERR", e.getMessage());
        }
    }

    private void loadJson() {
        try {
            File f = new File(Constants.Pinpoint.CALIBRATION_JSON);
            if (!f.exists()) return;
            StringBuilder sb = new StringBuilder();
            try (FileReader fr = new FileReader(f)) {
                int c; while ((c = fr.read()) != -1) sb.append((char)c);
            }
            JSONObject o = new JSONObject(sb.toString());
            trackWidth = o.getDouble("trackWidthMm");
            fwdOffset = o.getDouble("forwardOffsetMm");
            Constants.Pinpoint.TRACK_WIDTH_MM = trackWidth;
            Constants.Pinpoint.FORWARD_OFFSET_MM = fwdOffset;
        } catch (Exception e) {
            telemetry.addData("ERR", e.getMessage());
        }
    }
}
