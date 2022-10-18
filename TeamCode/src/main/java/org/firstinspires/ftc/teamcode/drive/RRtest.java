package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class RRtest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;
        Pose2d StartPose = new Pose2d(24,0, Math.toRadians(360));
        drive.setPoseEstimate(StartPose);
        Trajectory traj = drive.trajectoryBuilder(StartPose)
                .lineToLinearHeading(new Pose2d(48, 48, Math.toRadians(90)))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                .splineToSplineHeading(new Pose2d(24,-24, Math.toRadians(180)), Math.toRadians(0))
                        .build();
        drive.followTrajectory(traj);
        drive.followTrajectory(traj2);

    }
}
