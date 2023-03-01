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

@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
public class TeleopLM2 extends LinearOpMode {

    DcMotorEx motorLeft;
    DcMotorEx motorRight;

    Servo claw;

    DistanceSensor armRangeSensor;

    int commonModifier = 0;
    final int liftLow = 0;
    final int liftHigherThanLow = -700;
    final int liftMid = -1000;
    final int liftHigh = -1275;

    double driveSensitivity = .8;
    double turnSensitivity = .75;

    final double closed = 0.45;
    final double open = 0;

    final int extend = 1;
    final int rest = 0;
    final int retract = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pose2d StartPose = new Pose2d(0, 0, Math.toRadians(270));
        drive.setPoseEstimate(StartPose);

        claw = hardwareMap.get(Servo.class, "claw");

        motorLeft = hardwareMap.get(DcMotorEx.class, "LiftLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "LiftRight");

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
        }


        while (!isStopRequested()) {

            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y*driveSensitivity, -gamepad1.left_stick_x*driveSensitivity, -gamepad1.right_stick_x*turnSensitivity));
            drive.update();

            if(gamepad2.right_bumper) {
                claw.setPosition(closed);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if(gamepad2.left_bumper){
                claw.setPosition(open);
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            } else if(gamepad2.dpad_up) {
                // Set arm Position to High
                ArmPosition(liftHigh+commonModifier);
            } else if(gamepad2.dpad_down) {
                // Set arm Position to Low
                ArmPosition(liftLow+commonModifier);
            }else if(gamepad2.dpad_left) {
                // Set arm Position to Medium
                ArmPosition(liftMid+commonModifier);
            }else if(gamepad2.dpad_right) {
                // Set arm Position to a bit lower than High
                ArmPosition(liftHigherThanLow+commonModifier);
            }else if(gamepad2.a) {
                // Change the encoder modifier down
                commonModifier = commonModifier + 100;
                ArmPosition(motorLeft.getCurrentPosition() + 100);
            }else if(gamepad2.b) {
                // Change the encoder modifier up
                commonModifier = commonModifier - 100;
                ArmPosition(motorLeft.getCurrentPosition() - 100);
            }

            // Telemetry
            telemetry.addData("Roboracers Teleop for League Tournament", "");
            //telemetry.addData("range", String.format("%.01f mm", armRangeSensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("Gamepad 2 Left Stick Y", gamepad2.left_stick_y);
            telemetry.addData("Left Motor Power", motorLeft.getPower());
            telemetry.addData("Right Motor Power", motorRight.getPower());
            telemetry.addData("Left Motor Encoder Value", motorLeft.getCurrentPosition());
            telemetry.addData("Right Motor Encoder Value", motorRight.getCurrentPosition());
            telemetry.addData("Common Encoder Modifier", commonModifier);

            telemetry.update();

        }
    }

    // Function to set the arm position
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

    // Function to set the claw position
    public void claw(double posclaw) {
        claw.setPosition(posclaw);
        gamepad1.rumble(500);
        gamepad2.rumble(500);
    }
}
