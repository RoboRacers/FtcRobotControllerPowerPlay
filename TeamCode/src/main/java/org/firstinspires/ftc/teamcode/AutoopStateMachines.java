package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.RoadrunnerPointDataset;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.modules.opencv.SignalDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

@Config
@Autonomous(name = "AutoOp State Machines", group = "16481-Power-Play")
public class AutoopStateMachines extends LinearOpMode {

    public enum STATE_DRIVE{
        STATE_DRIVE_STOP,                    //drives stop
        STATE_DRIVE_FORWARD,                 //drives forward
        STATE_DRIVE_BACKWARD,                //drives backward
        STATE_DRIVE_STRAFE_LEFT,              //strafe left
        STATE_DRIVE_STRAFE_RIGHT,             //strafe right
        STATE_DRIVE_TURN_LEFT,                //rotate left
        STATE_DRIVE_TURN_RIGHT               //rotate right
    }
    public enum STATE_CLAW {
        STATE_CLAW_OPEN,                // claw opens
        STATE_CLAW_CLOSED              //claw closes
    }
    public enum STATE_ARM {
        STATE_ARM_LOW,               //raise arm height to medium pole
        STATE_ARM_MED,               //raise arm height to low pole
        STATE_ARM_HIGH,              //raise arm height to high pole
        STATE_ARM_PICK,              //raise arm height to pick up cone
        STATE_ARM_UP,                   //manual raise arm up
        STATE_ARM_DOWN,                 //manual lower arm down
        STATE_ARM_EXTEND_BACK,              //manual extend arm backward
        STATE_ARM_EXTEND_FRONT              // manual extend arm forward
    }
    public enum STATE_ROADRUNNER {
        //ToDo: Classify each preset
        STATE_ROADRUNNER_POS0,
        STATE_ROADRUNNER_POS1,          //Different roadrunner presets that are used in game
        STATE_ROADRUNNER_POS2,
        STATE_ROADRUNNER_POS3,
        STATE_ROADRUNNER_POS4,
        STATE_ROADRUNNER_POS5,
        STATE_ROADRUNNER_POS6,
        STATE_ROADRUNNER_POS7,
        STATE_ROADRUNNER_POS8,
        STATE_ROADRUNNER_POS9,
        STATE_ROADRUNNER_POS10,
        STATE_ROADRUNNER_POS11,
        STATE_ROADRUNNER_POS12,
        STATE_ROADRUNNER_POS13,
        STATE_ROADRUNNER_POS14,
        STATE_ROADRUNNER_POS15
    }
    public enum STATE_APRIL_TAG {
        STATE_APRIL_TAG_SEARCH,         //Searches for tag in each position on field
        STATE_APRIL_TAG_FOUND_0,        //Switches to this if tag is found at position 1 on field
        STATE_APRIL_TAG_FOUND_1,        //Switches to this if found at position 2 on field
        STATE_APRIL_TAG_FOUND_2         //Switches to this if found at position 3 on field
    }

    boolean CYCLE = false;
    public enum STATE_POSITION {
        STATE_POSITION_SP9,
        STATE_POSITION_SP0,
        STATE_POSITION_SP1,
        STATE_POSITION_SP2,
        STATE_POSITION_SP3
    }

    //Setting Current state for each section to desired starting state
    public STATE_DRIVE DriveState = STATE_DRIVE.STATE_DRIVE_STOP;
    public STATE_CLAW ClawState = STATE_CLAW.STATE_CLAW_OPEN;
    public STATE_ARM ArmState = STATE_ARM.STATE_ARM_PICK;
    public STATE_ROADRUNNER RoadrunnerState = STATE_ROADRUNNER.STATE_ROADRUNNER_POS0;
    public STATE_APRIL_TAG AprilTagState = STATE_APRIL_TAG.STATE_APRIL_TAG_SEARCH;
    public STATE_POSITION RobotPosition = STATE_POSITION.STATE_POSITION_SP9;


    // Timer to increment servo. Servo increment to next position when timer reach a set value
    // ElapsedTime timer_1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        // We are settting the robot position to Position 2, just because all of them are the same
        RobotPosition = STATE_POSITION.STATE_POSITION_SP2;
        while(!isStopRequested() && !opModeIsActive) {
            // Repeatedly update the signal ID with new detections from init to opmode start
            tagID = mySignalDetection.CheckSignal();
            telemetry.addData("# Tag ID: ", tagID);
            telemetry.addData("Position selected ", RobotPosition);
            telemetry.update();
        }

        camera.closeCameraDevice();

        // **********************************************************************
        // **********************************************************************
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        RoadrunnerPointDataset Trajectories = new RoadrunnerPointDataset(drive, (MultipleTelemetry) telemetry);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            switch (RobotPosition) {
                case STATE_POSITION_SP0:

                    drive.setPoseEstimate(Trajectories.S0_POS);
                    if(tagID == 0){ Trajectories.S0PP1(); }
                    else if (tagID == 1) { Trajectories.S0PP2(); }
                    else if (tagID == 2){  Trajectories.S0PP3(); }
                    RobotPosition = STATE_POSITION.STATE_POSITION_SP9;
                    break;
                case  STATE_POSITION_SP1:
                    drive.setPoseEstimate(Trajectories.S1_POS);
                    if(tagID == 0){ Trajectories.S1PP1(); }
                    else if (tagID == 1) { Trajectories.S1PP2(); }
                    else if (tagID == 2) { Trajectories.S1PP3(); }
                    RobotPosition = STATE_POSITION.STATE_POSITION_SP9;
                    break;
                case STATE_POSITION_SP2:
                    drive.setPoseEstimate(Trajectories.S2_POS);
                    if (tagID == 0) { Trajectories.S2PP1(); }
                    else if (tagID == 1) { Trajectories.S2PP2(); }
                    else if (tagID == 2) { Trajectories.S2PP3(); }
                    RobotPosition = STATE_POSITION.STATE_POSITION_SP9;
                    break;
                case STATE_POSITION_SP3:
                    drive.setPoseEstimate(Trajectories.S3_POS);
                    if(tagID == 0){ Trajectories.S3PP1(); }
                    else if (tagID == 1) { Trajectories.S3PP2(); }
                    else if (tagID == 2) { Trajectories.S3PP3(); }
                    RobotPosition = STATE_POSITION.STATE_POSITION_SP9;
                    break;
            }

            switch (DriveState) {
                case STATE_DRIVE_STOP:
                    //ToDo: Event to move to next state
                    if(gamepad1.left_stick_y > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_FORWARD;
                    if(gamepad1.left_stick_y < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_BACKWARD;
                    if(gamepad1.left_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_RIGHT;
                    if(gamepad1.left_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_LEFT;
                    if(gamepad1.right_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_RIGHT;
                    if(gamepad1.right_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_LEFT;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_DRIVE_FORWARD:
                    //ToDo: Event to move to next state
                    if(gamepad1.left_stick_y < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_BACKWARD;
                    if(gamepad1.left_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_RIGHT;
                    if(gamepad1.left_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_LEFT;
                    if(gamepad1.right_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_RIGHT;
                    if(gamepad1.right_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_LEFT;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_DRIVE_BACKWARD:
                    //ToDo: Event to move to next state
                    if(gamepad1.left_stick_y > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_FORWARD;
                    if(gamepad1.left_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_RIGHT;
                    if(gamepad1.left_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_LEFT;
                    if(gamepad1.right_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_RIGHT;
                    if(gamepad1.right_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_LEFT;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_DRIVE_STRAFE_RIGHT:
                    //ToDo: Event to move to next state
                    if(gamepad1.left_stick_y > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_FORWARD;
                    if(gamepad1.left_stick_y < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_BACKWARD;
                    if(gamepad1.left_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_LEFT;
                    if(gamepad1.right_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_RIGHT;
                    if(gamepad1.right_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_LEFT;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_DRIVE_STRAFE_LEFT:
                    //ToDo: Event to move to next state
                    if(gamepad1.left_stick_y > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_FORWARD;
                    if(gamepad1.left_stick_y < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_BACKWARD;
                    if(gamepad1.left_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_RIGHT;
                    if(gamepad1.right_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_RIGHT;
                    if(gamepad1.right_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_LEFT;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_DRIVE_TURN_RIGHT:
                    //ToDo: Event to move to next state
                    if(gamepad1.left_stick_y > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_FORWARD;
                    if(gamepad1.left_stick_y < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_BACKWARD;
                    if(gamepad1.left_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_RIGHT;
                    if(gamepad1.left_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_LEFT;
                    if(gamepad1.right_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_LEFT;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_DRIVE_TURN_LEFT:
                    //ToDo: Event to move to next state
                    if(gamepad1.left_stick_y > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_FORWARD;
                    if(gamepad1.left_stick_y < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_BACKWARD;
                    if(gamepad1.left_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_RIGHT;
                    if(gamepad1.left_stick_x < 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_STRAFE_LEFT;
                    if(gamepad1.right_stick_x > 0.1)
                        DriveState = STATE_DRIVE.STATE_DRIVE_TURN_RIGHT;
                    //ToDo: Action to be performed in this state.
                    break;
                default:
                    break;
            }
            switch (ClawState) {
                case STATE_CLAW_OPEN:
                    //ToDo: Event to move to next state
                    if(gamepad2.left_bumper)
                        ClawState = STATE_CLAW.STATE_CLAW_CLOSED;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_CLAW_CLOSED:
                    //ToDo: Event to move to next state
                    if(gamepad2.right_bumper)
                        ClawState = STATE_CLAW.STATE_CLAW_OPEN;
                    //ToDo: Action to be performed in this state.
                    break;
            }
            switch (ArmState) {
                case STATE_ARM_PICK:
                    //ToDo: Event to move to next state
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_LOW;
                    if(gamepad2.x)
                        ArmState = STATE_ARM.STATE_ARM_MED;
                    if(gamepad2.y)
                        ArmState = STATE_ARM.STATE_ARM_HIGH;
                    if(gamepad2.right_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_UP;
                    if(gamepad2.right_stick_y < 0.1)
                        ArmState = STATE_ARM.STATE_ARM_DOWN;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_FRONT;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_BACK;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ARM_LOW:
                    //ToDo: Event to move to next state
                    if(gamepad2.x)
                        ArmState = STATE_ARM.STATE_ARM_MED;
                    if(gamepad2.y)
                        ArmState = STATE_ARM.STATE_ARM_HIGH;
                    if(gamepad2.b)
                        ArmState = STATE_ARM.STATE_ARM_PICK;
                    if(gamepad2.right_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_UP;
                    if(gamepad2.right_stick_y < 0.1)
                        ArmState = STATE_ARM.STATE_ARM_DOWN;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_FRONT;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_BACK;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ARM_MED:
                    //ToDo: Event to move to next state
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_LOW;
                    if(gamepad2.y)
                        ArmState = STATE_ARM.STATE_ARM_HIGH;
                    if(gamepad2.b)
                        ArmState = STATE_ARM.STATE_ARM_PICK;
                    if(gamepad2.right_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_UP;
                    if(gamepad2.right_stick_y < 0.1)
                        ArmState = STATE_ARM.STATE_ARM_DOWN;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_FRONT;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_BACK;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ARM_HIGH:
                    //ToDo: Event to move to next state
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_LOW;
                    if(gamepad2.x)
                        ArmState = STATE_ARM.STATE_ARM_MED;
                    if(gamepad2.b)
                        ArmState = STATE_ARM.STATE_ARM_PICK;
                    if(gamepad2.right_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_UP;
                    if(gamepad2.right_stick_y < 0.1)
                        ArmState = STATE_ARM.STATE_ARM_DOWN;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_FRONT;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_BACK;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ARM_UP:
                    //ToDo: Event to move to next state
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_LOW;
                    if(gamepad2.x)
                        ArmState = STATE_ARM.STATE_ARM_MED;
                    if(gamepad2.y)
                        ArmState = STATE_ARM.STATE_ARM_HIGH;
                    if(gamepad2.right_stick_y < 0.1)
                        ArmState = STATE_ARM.STATE_ARM_DOWN;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_FRONT;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_BACK;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ARM_DOWN:
                    //ToDo: Event to move to next state
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_LOW;
                    if(gamepad2.x)
                        ArmState = STATE_ARM.STATE_ARM_MED;
                    if(gamepad2.y)
                        ArmState = STATE_ARM.STATE_ARM_HIGH;
                    if(gamepad2.right_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_UP;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_FRONT;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_BACK;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ARM_EXTEND_BACK:
                    //ToDo: Event to move to next state
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_LOW;
                    if(gamepad2.x)
                        ArmState = STATE_ARM.STATE_ARM_MED;
                    if(gamepad2.y)
                        ArmState = STATE_ARM.STATE_ARM_HIGH;
                    if(gamepad2.right_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_UP;
                    if(gamepad2.right_stick_y < 0.1)
                        ArmState = STATE_ARM.STATE_ARM_DOWN;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_FRONT;
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ARM_EXTEND_FRONT:
                    //ToDo: Event to move to next state
                    if(gamepad2.a)
                        ArmState = STATE_ARM.STATE_ARM_LOW;
                    if(gamepad2.x)
                        ArmState = STATE_ARM.STATE_ARM_MED;
                    if(gamepad2.y)
                        ArmState = STATE_ARM.STATE_ARM_HIGH;
                    if(gamepad2.right_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_UP;
                    if(gamepad2.right_stick_y < 0.1)
                        ArmState = STATE_ARM.STATE_ARM_DOWN;
                    if(gamepad2.left_stick_y > 0.1)
                        ArmState = STATE_ARM.STATE_ARM_EXTEND_BACK;
                    //ToDo: Action to be performed in this state.
                    break;
            }
            switch (RoadrunnerState) {
                case STATE_ROADRUNNER_POS0:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS1:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS2:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS3:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS4:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS5:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS6:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS7:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS8:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS9:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS10:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS11:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS12:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS13:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS14:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case STATE_ROADRUNNER_POS15:
                    //ToDo: Event to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
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
