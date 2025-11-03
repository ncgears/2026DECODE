package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import org.firstinspires.ftc.teamcode.constants.Constants;

/** FTCLib-backed drive using Motor wrappers and MecanumDrive. */
public class DriveSubsystem {
    private final Motor fl, fr, rl, rr;
    private final MecanumDrive mecanum;

    public DriveSubsystem(HardwareMap hw) {
        fl = new Motor(hw, Constants.Drive.FL);
        fr = new Motor(hw, Constants.Drive.FR);
        rl = new Motor(hw, Constants.Drive.RL);
        rr = new Motor(hw, Constants.Drive.RR);

        fl.setInverted(Constants.Drive.INVERT_FL);
        fr.setInverted(Constants.Drive.INVERT_FR);
        rl.setInverted(Constants.Drive.INVERT_RL);
        rr.setInverted(Constants.Drive.INVERT_RR);

        fl.setRunMode(Motor.RunMode.RawPower);
        fr.setRunMode(Motor.RunMode.RawPower);
        rl.setRunMode(Motor.RunMode.RawPower);
        rr.setRunMode(Motor.RunMode.RawPower);

        Motor.ZeroPowerBehavior z = Constants.Drive.ZERO_POWER_BRAKE
                ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT;
        fl.setZeroPowerBehavior(z); fr.setZeroPowerBehavior(z); rl.setZeroPowerBehavior(z); rr.setZeroPowerBehavior(z);

        mecanum = new MecanumDrive(fl, fr, rl, rr);
    }

    /** Robot-centric command: x=+right, y=+forward, rot=+CCW. */
    public void driveRobotCentric(double x, double y, double rot) {
        mecanum.driveRobotCentric(x, y, rot);
    }
    public void stop() { mecanum.stop(); }
}
