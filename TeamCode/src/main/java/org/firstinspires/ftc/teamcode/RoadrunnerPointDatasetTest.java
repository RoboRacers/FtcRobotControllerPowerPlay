package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RoadrunnerPointDataset;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.opencv.SignalDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@Autonomous(name = "Roadrunner test", group = "Test")
public class RoadrunnerPointDatasetTest extends LinearOpMode {
    public enum STATE_POSITION {
        STATE_POSITION_SP9,
        STATE_POSITION_SP0,
        STATE_POSITION_SP1,
        STATE_POSITION_SP2,
        STATE_POSITION_SP3
    }

    public STATE_POSITION RobotPosition = STATE_POSITION.STATE_POSITION_SP9;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RoadrunnerPointDataset Trajectories = new RoadrunnerPointDataset(drive, (MultipleTelemetry) telemetry);

        // **********************************************************************
        // April Tag detection Code
        // **********************************************************************
        OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        SignalDetection mySignalDetection = new SignalDetection(camera, (MultipleTelemetry) telemetry);
        mySignalDetection.openConnection();
        sleep(100);
        int tagID=-1;

        telemetry.addData("# Detecting AprilTag ","");
        telemetry.update();

        boolean Done = false;
        while(!Done && !isStopRequested()) {
            tagID = mySignalDetection.CheckSignal();
            if(gamepad1.x) RobotPosition = STATE_POSITION.STATE_POSITION_SP2;
            else if(gamepad1.y) RobotPosition = STATE_POSITION.STATE_POSITION_SP1;
            else if(gamepad1.a) RobotPosition = STATE_POSITION.STATE_POSITION_SP3;
            else if(gamepad1.b) RobotPosition = STATE_POSITION.STATE_POSITION_SP0;
            else if(gamepad1.right_bumper) Done = true;
            telemetry.addData("# Tag ID: ", tagID);
            telemetry.addData("Position selected ", RobotPosition);
            telemetry.update();
        }

        telemetry.addData("Position Confirmed ", RobotPosition);
        telemetry.update();

        camera.closeCameraDevice();

        // **********************************************************************
        // **********************************************************************

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;
        boolean rDone = false;
        while(opModeIsActive()){
            if (!rDone) {
                Trajectories.test();
                rDone = true;
            }
        }
    }

    /*
    public void ServoControl(Servo servoControl, double direction) {
        if(timer_1.milliseconds() >= 100) {
            servoControl.setPosition(servoControl.getPosition()+direction);
            timer_1.reset();
        }
    }*/
}
