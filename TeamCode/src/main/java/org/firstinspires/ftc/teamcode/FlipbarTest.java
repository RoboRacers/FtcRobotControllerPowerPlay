package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Flipbar Test")
public class FlipbarTest extends LinearOpMode {
    Servo flipbarLeft;
    Servo flipbarRight;


    double zero = 0.0;
    double one = 1.0;


    @Override
    public void runOpMode() {
        flipbarLeft = hardwareMap.get(Servo.class, "flipbarLeft");
        flipbarRight = hardwareMap.get(Servo.class, "flipbarRight");

        flipbarLeft.setDirection(Servo.Direction.FORWARD);
        flipbarRight.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.dpad_up) {
               flipbarLeft.setPosition(zero);
               flipbarRight.setPosition(-1);
            } else if(gamepad1.dpad_down) {
                flipbarLeft.setPosition(one);
                flipbarRight.setPosition(0);
            }
        }
    }
}