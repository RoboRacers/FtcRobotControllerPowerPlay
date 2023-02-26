package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TeleopLT extends LinearOpMode {

    DcMotorEx motorLeft;
    DcMotorEx motorRight;
    final int liftLow = 0;
    final int liftHigherThanLow = -700;
    final int liftMid = -1000;
    final int liftHigh = -1350;
    int targetPos = 20;

    Servo claw;
    Servo flipbarLeft;
    Servo flipbarRight;
    DistanceSensor armRangeSensor;

    final double closed = 0.7;
    final double open = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");

        motorLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "LiftRight");

        flipbarLeft = hardwareMap.get(Servo.class, "fbl");
        flipbarRight = hardwareMap.get(Servo.class, "fbr");

        flipbarLeft.setDirection(Servo.Direction.FORWARD);
        flipbarRight.setDirection(Servo.Direction.REVERSE);

        armRangeSensor = hardwareMap.get(DistanceSensor.class, "armRange");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()) {
            claw(open);
            flipbarLeft.setPosition(0.5);
            flipbarRight.setPosition(0.5);
        }


        while (!isStopRequested()) {


            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*.80, -gamepad1.left_stick_x*.80, -gamepad1.right_stick_x*.75));
            drive.update();
            flipbarLeft.setPosition(0.5);
            flipbarRight.setPosition(0.5);
            if(gamepad2.right_bumper) {
                claw.setPosition(closed);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if(gamepad2.left_bumper){
                claw.setPosition(open);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if(gamepad2.dpad_up) {
                ArmPosition(liftHigh);
            } else if(gamepad2.dpad_down) {
                ArmPosition(liftLow);
            }else if(gamepad2.dpad_left) {
                ArmPosition(liftMid);
            }else if(gamepad2.dpad_right) {
                ArmPosition(liftHigherThanLow);
            }else if(gamepad2.b) {
                ArmPosition(motorLeft.getCurrentPosition() + 10);
            }else if(gamepad2.a) {
                ArmPosition(motorLeft.getCurrentPosition() - 10);
            }else if (gamepad2.left_stick_y < -0.5) {
                motorLeft.setPower(-0.1);
                motorRight.setPower(-0.1);
            }else if (gamepad2.left_stick_y > 0.5) {
                motorLeft.setPower(0.1);
                motorRight.setPower(0.1);
            } else if (gamepad2.left_stick_y == 0) {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }

            telemetry.addData("Gamepad 2 Left Stick X", gamepad2.left_stick_y);
            telemetry.addData("Left Motor", motorLeft.getPower());
            telemetry.addData("Right Motor", motorRight.getPower());

            telemetry.update();
        }
    }
    public void ArmPosition(int pos) {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight.setTargetPosition(pos);
        motorLeft.setTargetPosition(pos);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(.75);
        motorRight.setPower(.75);
    }

    public void claw(double posclaw) {
        claw.setPosition(posclaw);
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }


}
