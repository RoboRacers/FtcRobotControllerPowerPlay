package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.gaeldrive.AlphaUpdate;

@TeleOp(name = "Teleop For Testing Alpha Update", group = "16481-Power-Play")
public class TeleopUpdateA extends LinearOpMode {

    ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Drive Setup
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (opModeInInit()) {
            
        }

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            drive.update();

            Pose2d robotPoseEstimate = AlphaUpdate.mecanumPoseDeltaCalc(drive, gamepad1, loopTime);

            loopTime.reset();

            // Telemetry
            telemetry.addData("Roboracers Teleop for Testing Update A", "");
            telemetry.addData("loopTime Resolution", loopTime.getResolution());
            telemetry.addData("x", robotPoseEstimate.getX());
            telemetry.addData("y", robotPoseEstimate.getY());
            telemetry.addData("heading", robotPoseEstimate.getHeading());
            telemetry.update();

        }
    }

}