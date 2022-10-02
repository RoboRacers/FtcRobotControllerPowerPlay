package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name = "stateMachine", group = "Concept")
public class workStateMachines extends LinearOpMode {
    public enum States{
       STATE_0, //innit
        clawOpen, // claw opens
        clawClosed, //claw closes
        Forward, //drives forward
        Backward, //drives backward
        strafeLeft, //strafe left
        strafeRight, //strafe right
        turnLeft, //rotate left
        turnRight, //rotate right
        heightMed, //raise arm height to medium pole
        heightLow, //raise arm height to low pole
        heightHigh,//raise arm height to high pole
        heightPick,//raise arm height to pick up cone
        armUp, //manual raise arm up
        armDown,//manual lower arm down
        extendBack, //manual extend arm backward
        extendFront // manual extend arm forward
    }

    //Setting Current state to STATE_0 that is init state.
    public States CurrentState = States.STATE_0;

    // Timer to increment servo. Servo increment to next position when timer reach a set value
    ElapsedTime timer_1 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Define class members
    Servo servo_1;
    Servo servo_2;
    TouchSensor button_1;
    TouchSensor button_2;
    TouchSensor button_3;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo_1 = hardwareMap.get(Servo.class, "servo1");
        servo_2 = hardwareMap.get(Servo.class, "servo2");
        button_1 = hardwareMap.get(TouchSensor.class, "b1");
        button_2 = hardwareMap.get(TouchSensor.class, "b2 ");
        button_3 = hardwareMap.get(TouchSensor.class, "b3");

        timer_1.startTime();

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            switch(CurrentState) {
                case STATE_0:
                    //ToDo: Evernt to move to next state
                    //ToDo: Action to be performed in this state.
                    break;
                case clawOpen:
                    // Action performed in this state
                    servo_1.setPosition(0.00);
                    servo_2.setPosition(0.00);
                    // Event that move create transaction to next state
                    if(button_2.isPressed() == true){
                        CurrentState = States.clawClosed;
                    }

                    break;
                case clawClosed:
                    servo_1.setPosition(0.50);
                    servo_2.setPosition(0.50);
                    if(button_2.isPressed() == true){
                        CurrentState = States.clawOpen;
                    }
                    break;
                default:
                    break;
            }
        }
    }

    public void ServoControl(Servo servoControl, double direction) {
        if(timer_1.milliseconds() >= 100) {
            servoControl.setPosition(servoControl.getPosition()+direction);
            timer_1.reset();
        }
    }
}
