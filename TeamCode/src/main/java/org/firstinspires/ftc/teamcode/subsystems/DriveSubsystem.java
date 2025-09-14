package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class DriveSubsystem {
    private final DcMotorEx fl, fr, rl, rr;

    public DriveSubsystem(HardwareMap hw) {
        fl = hw.get(DcMotorEx.class, Constants.Drive.FL);
        fr = hw.get(DcMotorEx.class, Constants.Drive.FR);
        rl = hw.get(DcMotorEx.class, Constants.Drive.RL);
        rr = hw.get(DcMotorEx.class, Constants.Drive.RR);

        fl.setDirection(Constants.Drive.INVERT_FL ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        fr.setDirection(Constants.Drive.INVERT_FR ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rl.setDirection(Constants.Drive.INVERT_RL ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        rr.setDirection(Constants.Drive.INVERT_RR ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        if (Constants.Drive.RUN_WITHOUT_ENCODERS) {
            fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        DcMotor.ZeroPowerBehavior z = Constants.Drive.ZERO_POWER_BRAKE ?
                DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        fl.setZeroPowerBehavior(z); fr.setZeroPowerBehavior(z); rl.setZeroPowerBehavior(z); rr.setZeroPowerBehavior(z);

        for (LynxModule hub : hw.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(Constants.Drive.REV_BULK_CACHE_AUTO
                    ? LynxModule.BulkCachingMode.AUTO : LynxModule.BulkCachingMode.OFF);
        }
    }

    public void driveRobotCentric(double x, double y, double rot) {
        double flp = y + x + rot;
        double frp = y - x - rot;
        double rlp = y - x + rot;
        double rrp = y + x - rot;
        double max = Math.max(1.0, Math.max(Math.abs(flp),
                Math.max(Math.abs(frp), Math.max(Math.abs(rlp), Math.abs(rrp)))));
        fl.setPower(flp / max); fr.setPower(frp / max); rl.setPower(rlp / max); rr.setPower(rrp / max);
    }
}
