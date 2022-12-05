package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class TeleopLM2 extends LinearOpMode {

    DcMotorEx motorLeft;
    DcMotorEx motorRight;
    final int liftLow = 0;
    final int liftHigherThanLow = -600;
    final int liftMid = -900;
    final int liftHigh = -1200;

    int lower;
    int pos;
    int higher;
    Servo claw;
    final double closed = 0.7;
    final double open =0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = hardwareMap.get(Servo.class, "claw");

        motorLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "LiftRight");
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeInInit()) {
            claw(open);
        }
        while (!isStopRequested()) {
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*.80, -gamepad1.left_stick_x*.80, -gamepad1.right_stick_x*.7)); drive.update();
            if(gamepad2.right_bumper) {
                claw(closed);
            } else if(gamepad2.left_bumper){
                claw(open);
            } else if(gamepad2.dpad_up) {
                ArmPosition(liftHigh);
            } else if(gamepad2.dpad_down) {
                ArmPosition(liftLow);
            }else if(gamepad2.dpad_left) {
                ArmPosition(liftMid);
            }else if(gamepad2.dpad_right) {
                ArmPosition(liftHigherThanLow);
            }else if(gamepad2.b) {
                ArmManualRetract();
            }else if(gamepad2.a) {
                ArmManualExtend();
            }
        }
    }
    public void ArmPosition(int pos) {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight.setTargetPosition(pos);
        motorLeft.setTargetPosition(pos);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(1);
        motorRight.setPower(1);
    }
    public void ArmManualRetract() {
        pos = motorLeft.getCurrentPosition();
        lower = pos + 50;
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight.setTargetPosition(lower);
        motorLeft.setTargetPosition(lower);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(1);
        motorRight.setPower(1);
    }
    public void ArmManualExtend() {
        pos = motorLeft.getCurrentPosition();
        higher = pos - 50;
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorRight.setTargetPosition(higher);
        motorLeft.setTargetPosition(higher);
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(1);
        motorRight.setPower(1);
    }
    public void claw(double posclaw) {
        claw.setPosition(posclaw);
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }
}
