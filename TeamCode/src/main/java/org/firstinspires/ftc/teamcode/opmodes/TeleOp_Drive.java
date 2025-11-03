package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUHeading;
import org.firstinspires.ftc.teamcode.subsystems.HeadingProvider;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.util.Shaping;

import org.json.JSONObject;
import java.io.File;
import java.io.FileReader;

@TeleOp(name = "TeleOp_Drive (Autoload Pinpoint)", group = "Main")
//@Disabled
public class TeleOp_Drive extends OpMode {
    private DriveSubsystem drive;
    private HeadingProvider imuHeading;
    private boolean useField = Constants.Drive.FIELD_CENTRIC_DEFAULT;
    private SlewRateLimiter slewX, slewY, slewR;

    @Override public void init() {
        loadPinpointCalibrationIfPresent();
        drive = new DriveSubsystem(hardwareMap);
        imuHeading = new IMUHeading(hardwareMap);
        slewX = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewY = new SlewRateLimiter(Constants.Drive.SLEW_TRANSLATION);
        slewR = new SlewRateLimiter(Constants.Drive.SLEW_ROTATION);
    }

    private void loadPinpointCalibrationIfPresent() {
        try {
            File f = new File(Constants.Pinpoint.CALIBRATION_JSON);
            if (!f.exists()) return;
            StringBuilder sb = new StringBuilder();
            try (FileReader fr = new FileReader(f)) { int c; while ((c = fr.read()) != -1) sb.append((char)c); }
            JSONObject o = new JSONObject(sb.toString());
            Constants.Pinpoint.TRACK_WIDTH_MM = o.optDouble("trackWidthMm", Constants.Pinpoint.TRACK_WIDTH_MM);
            Constants.Pinpoint.FORWARD_OFFSET_MM = o.optDouble("forwardOffsetMm", Constants.Pinpoint.FORWARD_OFFSET_MM);
            telemetry.addLine("Pinpoint calib loaded");
        } catch (Exception e) {
            telemetry.addLine("Pinpoint calib load failed");
        }
    }

    @Override public void loop() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        lx = Shaping.translate(lx);
        ly = Shaping.translate(ly);
        rx = Shaping.rotate(rx);

        double scale = gamepad1.left_bumper ? Constants.Drive.PRECISION_SCALE : 1.0;
        lx *= scale; ly *= scale; if (Constants.Drive.PRECISION_SCALES_ROTATION) rx *= scale;

        double x = lx, y = ly;
        if (useField) {
            double h = imuHeading.getHeadingRad();
            double cos = Math.cos(-h), sin = Math.sin(-h);
            double tx = x * cos - y * sin;
            double ty = x * sin + y * cos;
            x = tx; y = ty;
        }
        double rot = rx * (Constants.Drive.ROTATION_CW_IS_POSITIVE ? -1.0 : 1.0);

        if (Constants.Drive.SLEW_ENABLED) {
            x = slewX.calculate(x); y = slewY.calculate(y); rot = slewR.calculate(rot);
        }
        drive.driveRobotCentric(x, y, rot);
        telemetry.update();
    }
}
