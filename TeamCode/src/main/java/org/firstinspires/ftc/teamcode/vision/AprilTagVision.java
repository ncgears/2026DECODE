package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilTagVision {
    public final VisionPortal portal;
    public final AprilTagLibrary alib;
    public final AprilTagProcessor atag;

    public AprilTagVision(HardwareMap hw) {
        alib = new AprilTagLibrary.Builder()
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
        atag = new AprilTagProcessor.Builder()
                .setTagLibrary(alib)
                .build();

        WebcamName webcam = hw.get(WebcamName.class, Constants.Vision.WEBCAM_NAME);
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(atag)
                .setCameraResolution(new Size(Constants.Vision.STREAM_WIDTH, Constants.Vision.STREAM_HEIGHT))
                .build();
    }
}
