package org.firstinspires.ftc.teamcode.util;

import android.content.Context;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class DashboardInit {

    @OnCreate
    public static void init(Context context) {
        // Flip your constant to show/hide the Dashboard Enable/Disable OpMode
        if (!Constants.Global.ENABLE_DASHBOARD_OPMODE) {
            FtcDashboard.suppressOpMode();
        }
    }
}
