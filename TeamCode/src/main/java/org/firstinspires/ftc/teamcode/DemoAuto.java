package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.gaeldrive.AlphaUpdate;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "Demo Autonomous", group = "16481-Power-Play")
public class DemoAuto extends LinearOpMode {

    Pose2d robotPoseEstimate = new Pose2d(0,0,0);

    Trajectory traj1;

    @Override
    public void runOpMode() throws InterruptedException {
        // Drive Setup
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(robotPoseEstimate);
        while (opModeInInit()) {

        }

        while (!isStopRequested()) {

            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            drive.update();



            // Telemetry
            telemetry.addData("RoboRacers Teleop for Testing Update A", "");

        }
    }

}