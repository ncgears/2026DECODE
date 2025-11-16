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
@Disabled
public class Calibration_Pinpoint_Drive extends OpMode {

    // Drive
    private DriveSubsystem drive;
    private HeadingProvider imuHeading;
    private boolean fieldCentric = true;

    // Baseline asymmetric geometry from constants (used to preserve left:right ratio)
    private final double baseLeft  = Math.abs(Constants.Pinpoint.X_POD_SIDE_OFFSET_MM);   // left pod (+Y)
    private final double baseRight = Math.abs(Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM);   // right pod (-Y)

    // Tuning values (mm). These are what you see / edit on the DS.
    // Track width is left+right; forward offset is distance from CoR to both pods.
    private double trackWidth = baseLeft + baseRight;
    private double fwdOffset  =
            0.5 * (Constants.Pinpoint.X_POD_FORWARD_OFFSET_MM +
                    Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM);

    // Button edge detection
    private boolean lastA, lastB, lastX, lastY, lastBack;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap);
        imuHeading = new IMUHeading(hardwareMap);

        // Load existing JSON if present (so you can tweak previous calibration)
        loadJson();

        telemetry.addLine("Calibration_Pinpoint_Drive ready.");
        telemetry.addLine("Use sticks to drive; D-pad to tune; A=Save/B=Load/X=Reset.");
    }

    @Override
    public void loop() {
        // --- Step size selection for tuning ---
        double step = 1.0;
        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            step = 0.5;
        } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            step = 5.0;
        }

        // --- Adjust tuning values with D-pad ---
        if (gamepad1.dpad_left)  trackWidth -= step;
        if (gamepad1.dpad_right) trackWidth += step;
        if (gamepad1.dpad_down)  fwdOffset  -= step;
        if (gamepad1.dpad_up)    fwdOffset  += step;

        // Prevent nonsense values
        if (trackWidth < 50.0) trackWidth = 50.0;
        if (fwdOffset  < -200.0) fwdOffset = -200.0;
        if (fwdOffset  >  200.0) fwdOffset =  200.0;

        // --- Button edge detection ---
        boolean a    = gamepad1.a;
        boolean b    = gamepad1.b;
        boolean x    = gamepad1.x;
        boolean y    = gamepad1.y;
        boolean back = gamepad1.back;

        boolean aEdge    = a && !lastA;
        boolean bEdge    = b && !lastB;
        boolean xEdge    = x && !lastX;
        boolean yEdge    = y && !lastY;
        boolean backEdge = back && !lastBack;

        lastA = a; lastB = b; lastX = x; lastY = y; lastBack = back;

        if (aEdge) saveJson();
        if (bEdge) loadJson();
        if (xEdge) resetToDefaults();
        if (yEdge) fieldCentric = !fieldCentric;
        if (backEdge && imuHeading instanceof IMUHeading) {
            ((IMUHeading) imuHeading).zeroHeading();
        }

        // --- Reflect tuned values into Constants.Pinpoint for the rest of the codebase ---
        applyToConstants();

        // --- Drive like TeleOp_Drive (g1 sticks) ---
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        lx = Shaping.translate(lx);
        ly = Shaping.translate(ly);
        rx = Shaping.rotate(rx);

        double scale = gamepad1.left_bumper ? Constants.Drive.PRECISION_SCALE : 1.0;
        lx *= scale; ly *= scale;
        if (Constants.Drive.PRECISION_SCALES_ROTATION) rx *= scale;

        double xDrive = lx;
        double yDrive = ly;

        if (fieldCentric) {
            double h = imuHeading.getHeadingRad();
            double cos = Math.cos(-h), sin = Math.sin(-h);
            double tx = xDrive * cos - yDrive * sin;
            double ty = xDrive * sin + yDrive * cos;
            xDrive = tx; yDrive = ty;
        }

        double rot = rx * (Constants.Drive.ROTATION_CW_IS_POSITIVE ? -1.0 : 1.0);
        drive.driveRobotCentric(xDrive, yDrive, rot);

        // --- Telemetry ---
        telemetry.addData("TrackWidth (mm)", "%.1f", trackWidth);
        telemetry.addData("ForwardOffset (mm)", "%.1f", fwdOffset);
        telemetry.addData("FieldCentric", fieldCentric);
        telemetry.addLine("LB=0.5mm (fine)  RB=5.0mm (coarse)  none=1.0mm");
        telemetry.addLine("A=Save  B=Load  X=Reset  Y=FC toggle  BACK=Zero IMU");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (drive != null) drive.stop();
    }

    // ---- Apply tuned values into Constants.Pinpoint (asymmetric pods) ----
    private void applyToConstants() {
        // 1) Forward offsets: keep both pods at the same x distance from CoR.
        Constants.Pinpoint.X_POD_FORWARD_OFFSET_MM = fwdOffset;
        Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM = fwdOffset;

        // 2) Side offsets: scale baseline asymmetric geometry to match desired trackWidth.
        double baseTrack = baseLeft + baseRight;
        if (baseTrack <= 1e-6) return;

        double scale = trackWidth / baseTrack;
        double newLeft  = baseLeft  * scale;
        double newRight = baseRight * scale;

        // Left pod is +Y, right pod is -Y
        Constants.Pinpoint.X_POD_SIDE_OFFSET_MM =  newLeft;
        Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM = -newRight;

        // 3) Keep the convenience TRACK_WIDTH_MM in sync.
        Constants.Pinpoint.TRACK_WIDTH_MM =
                Math.abs(Constants.Pinpoint.X_POD_SIDE_OFFSET_MM -
                        Constants.Pinpoint.Y_POD_SIDE_OFFSET_MM);
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
            try (FileWriter fw = new FileWriter(f)) {
                fw.write(o.toString());
            }
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
                int c;
                while ((c = fr.read()) != -1) sb.append((char) c);
            }
            JSONObject o = new JSONObject(sb.toString());
            trackWidth = o.optDouble("trackWidthMm", trackWidth);
            fwdOffset  = o.optDouble("forwardOffsetMm", fwdOffset);
            telemetry.addLine("Loaded calibration JSON");
        } catch (Exception e) {
            telemetry.addData("ERR loadJson", e.getMessage());
        }
    }

    // Reset tuning values to compile-time defaults from Constants.Pinpoint.
    private void resetToDefaults() {
        trackWidth = baseLeft + baseRight;
        fwdOffset  = 0.5 * (Constants.Pinpoint.X_POD_FORWARD_OFFSET_MM +
                Constants.Pinpoint.Y_POD_FORWARD_OFFSET_MM);
    }
}
