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

    ElapsedTime loopTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    Pose2d robotPoseEstimate = new Pose2d(0,0,0);

    double botX = 0;
    double botY = 0;
    double botHeading = 0;

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
            loopTime.reset();

            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            drive.update();

            Pose2d PoseDelta = AlphaUpdate.mecanumPoseDeltaCalc(drive, gamepad1, loopTime);

            botX = botX + PoseDelta.getX();
            botY = botY + PoseDelta.getY();
            botHeading = botHeading + PoseDelta.getHeading();


            // Telemetry
            telemetry.addData("RoboRacers Teleop for Testing Update A", "");
            telemetry.addData("loopTime Resolution", loopTime.getResolution());
            Pose2d accuratePoseEstimate = drive.getPoseEstimate();
            telemetry.addData("accurate x", accuratePoseEstimate.getX());
            telemetry.addData("accurate y", accuratePoseEstimate.getY());
            telemetry.addData("accurate heading", accuratePoseEstimate.getHeading());

            telemetry.addData("estimated x", botX);
            telemetry.addData("estimated y", botY);
            telemetry.addData("estimated heading", botHeading);
            telemetry.update();

            loopTime.reset();
        }
    }

}