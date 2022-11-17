package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.modules.opencv.ConeDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;


@Config
@TeleOp(name = "TeleOp State Machines", group = "16481-Power-Play")
public class TeleopStateMachines extends LinearOpMode {

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

    /**
     * Setting Current state for each section to desired starting state
     */
    public STATE_DRIVE DriveState = STATE_DRIVE.STATE_DRIVE_STOP;
    public STATE_CLAW ClawState = STATE_CLAW.STATE_CLAW_OPEN;
    public STATE_ARM ArmState = STATE_ARM.STATE_ARM_PICK;
    public STATE_ROADRUNNER RoadrunnerState = STATE_ROADRUNNER.STATE_ROADRUNNER_POS0;
    public STATE_APRIL_TAG AprilTagState = STATE_APRIL_TAG.STATE_APRIL_TAG_SEARCH;

    /**
     * Lift Level constants
     */
    final int LIFT_LEVEL_0 = 0;
    final int LIFT_LEVEL_1 = -800;
    final int LIFT_LEVEL_2 = -1000;
    final int LIFT_LEVEL_3 = -1200;

    /**
     * Timer to increment servo. Servo increment to next position when timer reach a set value
     */

   // ElapsedTime timer_1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    DcMotor motorLeft;
    DcMotor motorRight;
    Servo claw;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /** **********************************************************************
         * cone detection Code
         **********************************************************************/
        OpenCvCamera camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.
                get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ConeDetection myConeDetection = new ConeDetection(camera);

        /** **********************************************************************
         * Hardware map for Claw and Lift
         **********************************************************************/
        claw = hardwareMap.get(Servo.class, "claw");
        motorLeft  = hardwareMap.get(DcMotor.class, "LiftLeft");
        motorRight  = hardwareMap.get(DcMotor.class, "LiftRight");

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            /** **********************************************************************
             * Drive Code
             **********************************************************************/
            drive.setWeightedDrivePower(
                 new Pose2d(
                     -gamepad1.left_stick_y*.75,
                     -gamepad1.left_stick_x*.75, //imperfect strafing fix, must be tuned for new drivetrain
                     -gamepad1.right_stick_x*.75
                 )
            );
            drive.update();
            /** **********************************************************************
             *
             **********************************************************************/
             switch(DriveState) {
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
            /** **********************************************************************
             *
             **********************************************************************/
            switch (ClawState) {
                case STATE_CLAW_OPEN:
                    claw.setPosition(0);
                    if(gamepad2.left_bumper) {
                        ClawState = STATE_CLAW.STATE_CLAW_CLOSED;
                        gamepad1.rumble(500);
                        gamepad2.rumble(500);
                    }
                    break;
                case STATE_CLAW_CLOSED:
                    claw.setPosition(1);
                    gamepad1.rumble(500);
                    if(gamepad2.right_bumper) {
                        ClawState = STATE_CLAW.STATE_CLAW_OPEN;
                        gamepad1.rumble(500);
                        gamepad2.rumble(500);
                    }
                    break;
            }
            /** **********************************************************************
             *
             **********************************************************************/
            switch (ArmState) {
                case STATE_ARM_PICK:
                    setArmPosition(0);
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
                    break;
                case STATE_ARM_LOW:
                    setArmPosition(50);
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
                    break;
                case STATE_ARM_MED:
                    setArmPosition(-500);
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
                    break;
                case STATE_ARM_HIGH:
                    setArmPosition(-1000);
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
                    break;
                case STATE_ARM_UP:
                case STATE_ARM_DOWN:
                    setArmPosition(0);
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
                    break;
                case STATE_ARM_EXTEND_BACK:
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
                    break;
                case STATE_ARM_EXTEND_FRONT:
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
                    break;
            }
            /** **********************************************************************
             *
             **********************************************************************/
            switch (RoadrunnerState){
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

    public void setArmPosition(int target)
    {
        if (motorLeft.isBusy() || motorRight.isBusy())
            return;
        if ((ArmState== STATE_ARM.STATE_ARM_DOWN) || (ArmState == STATE_ARM.STATE_ARM_UP)) {
            motorLeft.setPower(gamepad2.right_stick_y);
            motorRight.setPower(gamepad2.right_stick_y);
        }  else  {
            motorLeft.setPower(0);
            motorRight.setPower(0);

            motorRight.setTargetPosition(target);
            motorLeft.setTargetPosition(target);

            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorLeft.setPower(.5);
            motorRight.setPower(.5);
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
