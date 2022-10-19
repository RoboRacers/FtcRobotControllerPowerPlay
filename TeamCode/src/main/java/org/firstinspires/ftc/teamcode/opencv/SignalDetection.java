package org.firstinspires.ftc.teamcode.opencv;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

public class SignalDetection {
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    public AprilTagDetection tagOfInterest = null;

    public SignalDetection(OpenCvCamera camera) {
        camera.setPipeline(aprilTagDetectionPipeline);

        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.045, fx, fy, cx, cy);
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0) {
            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 0 || tag.id == 1 || tag.id == 2) {
                    tagOfInterest = tag;
                    break;
                }
            }
        }
    }

    public int CheckSignal()
    {
        return tagOfInterest.id;
    }
}