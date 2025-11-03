package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUHeading;
import org.firstinspires.ftc.teamcode.subsystems.HeadingProvider;
import org.firstinspires.ftc.teamcode.util.Shaping;

import org.json.JSONObject;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

/**
 * Calibration_Pinpoint_Drive
 * - Lets you DRIVE the robot while tuning Pinpoint values.
 * - Adjust TrackWidth and ForwardOffset with D-pad (LB=fine, RB=coarse).
 * - Save/Load to JSON on the RC so TeleOp can auto-load on INIT.
 *
 * Controls (g1):
 *  - D-pad LEFT/RIGHT: TrackWidth -/+
 *  - D-pad DOWN/UP:   ForwardOffset -/+
 *  - LB = fine step (0.5 mm), RB = coarse step (5.0 mm), none = 1.0 mm
 *  - A = Save, B = Load, X = Reset defaults
 *  - Y = Toggle Field-Centric drive
 *  - BACK = Zero IMU heading
 *  - Sticks drive like TeleOp_Drive (left XY, right X rotate)
 */
@TeleOp(name = "Calibration_Pinpoint_Drive", group = "Tuning")
//@Disabled
public class Calibration_Pinpoint_Drive extends OpMode {

    // Drive
    private DriveSubsystem drive;
    private HeadingProvider imuHeading;
    private boolean fieldCentric = true;

    // Tuning values (mirrors Constants.Pinpoint; we reflect to constants so TeleOp sees them)
    private double trackWidth = Constants.Pinpoint.TRACK_WIDTH_MM;
    private double fwdOffset  = Constants.Pinpoint.FORWARD_OFFSET_MM;

    @Override public void init() {
        drive = new DriveSubsystem(hardwareMap);
        imuHeading = new IMUHeading(hardwareMap);

        // Load existing JSON if present (so you can tweak)
        loadJson();

        telemetry.addLine("Calibration_Pinpoint_Drive ready. Use sticks to drive; D-pad to tune; A=Save/B=Load/X=Reset.");
    }

    @Override public void loop() {
        // --- Stepsize selection ---
        double step = 1.0;
        if (gamepad1.left_bumper)  step = 0.5;
        if (gamepad1.right_bumper) step = 5.0;

        // --- Adjust values ---
        if (gamepad1.dpad_left)  trackWidth -= step;
        if (gamepad1.dpad_right) trackWidth += step;
        if (gamepad1.dpad_down)  fwdOffset  -= step;
        if (gamepad1.dpad_up)    fwdOffset  += step;

        // --- Save / Load / Reset ---
        if (gamepad1.a) saveJson();
        if (gamepad1.b) loadJson();
        if (gamepad1.x) { trackWidth = 150.0; fwdOffset = 0.0; }

        // Reflect into Constants so TeleOp (or anything else) sees current values immediately
        Constants.Pinpoint.TRACK_WIDTH_MM = trackWidth;
        Constants.Pinpoint.FORWARD_OFFSET_MM = fwdOffset;

        // --- Drive controls ---
        if (gamepad1.y) fieldCentric = !fieldCentric;
        if (gamepad1.back) imuHeading.zeroHeading();

        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        lx = Shaping.translate(lx);
        ly = Shaping.translate(ly);
        rx = Shaping.rotate(rx);

        double x = lx, y = ly;
        if (fieldCentric) {
            double h = imuHeading.getHeadingRad();
            double cos = Math.cos(-h), sin = Math.sin(-h);
            double tx = x * cos - y * sin;
            double ty = x * sin + y * cos;
            x = tx; y = ty;
        }
        double rot = rx * (Constants.Drive.ROTATION_CW_IS_POSITIVE ? -1.0 : 1.0);
        drive.driveRobotCentric(x, y, rot);

        // --- Telemetry ---
        telemetry.addData("TrackWidth (mm)", "%.1f", trackWidth);
        telemetry.addData("ForwardOffset (mm)", "%.1f", fwdOffset);
        telemetry.addData("FieldCentric", fieldCentric);
        telemetry.addLine("LB=0.5mm  (fine)   RB=5.0mm (coarse)   none=1.0mm");
        telemetry.addLine("A=Save  B=Load  X=Reset  Y=FC toggle  BACK=Zero IMU");
        telemetry.update();
    }

    @Override public void stop() {
        if (drive != null) drive.stop();
    }

    // ---- JSON persistence helpers ----
    private void saveJson() {
        try {
            File f = new File(Constants.Pinpoint.CALIBRATION_JSON);
            File parent = f.getParentFile();
            if (parent != null && !parent.exists()) parent.mkdirs();
            JSONObject o = new JSONObject();
            o.put("trackWidthMm", trackWidth);
            o.put("forwardOffsetMm", fwdOffset);
            try (FileWriter fw = new FileWriter(f)) { fw.write(o.toString()); }
            telemetry.addLine("Saved calibration JSON");
        } catch (Exception e) {
            telemetry.addData("ERR saveJson", e.getMessage());
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
            trackWidth = o.optDouble("trackWidthMm", trackWidth);
            fwdOffset  = o.optDouble("forwardOffsetMm", fwdOffset);
            telemetry.addLine("Loaded calibration JSON");
        } catch (Exception e) {
            telemetry.addData("ERR loadJson", e.getMessage());
        }
    }
}
