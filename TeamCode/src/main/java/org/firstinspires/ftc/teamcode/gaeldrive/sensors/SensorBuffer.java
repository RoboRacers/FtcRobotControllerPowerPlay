package org.firstinspires.ftc.teamcode.gaeldrive.sensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

public class SensorBuffer {
    /* Flags for user to change */
    public boolean hasTrackingWheels = true;

    StandardTrackingWheelLocalizer trackingWheelLocalizer;
    Pose2d trackingWheelPose;
    double trackingWheelWeight = 1.1;

    public SensorBuffer(HardwareMap hardwareMap){
        if (hasTrackingWheels){
            this.trackingWheelLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        }
    }

    public void update(){
        if (hasTrackingWheels){
            this.trackingWheelLocalizer.update();
            this.trackingWheelPose = this.trackingWheelLocalizer.getPoseEstimate();
        }
    }
}
