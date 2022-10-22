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
public class stateMachine extends LinearOpMode {
    public enum States{
        STATE_0, // This is first state at on opMode init. No buttons are pressed.
        STATE_1, // In this state servo-2 is moving CW direction and Servo-1 is ideal. This will be first state.
        STATE_2, // This is loop state where Servo-1 cw until hit button 1, Servo-2 cw
        STATE_3  // In this state both servos are moving ccw until s2 hits button 2
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

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo_1 = hardwareMap.get(Servo.class, "servo1");
        servo_2 = hardwareMap.get(Servo.class, "servo2");
        button_1 = hardwareMap.get(TouchSensor.class, "b1");
        button_2 = hardwareMap.get(TouchSensor.class, "b2 ");

        timer_1.startTime();

        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive()){
            switch(CurrentState) {
                case STATE_0:
                    //first state at on opMode init. No buttons are pressed.
                    // Set both the servo to stating position.
                    servo_1.setPosition(0.00);
                    servo_2.setPosition(0.00);
                    // Manually press the button-1 to start the state machine by moving to next state
                    // We will never come back to this state.
                   if( button_1.isPressed() == true) {
                       CurrentState = States.STATE_1;
                   }
                    break;
                case STATE_1:
                    //Servo 1 idle, Servo 2 cw until button 2 hit
                    ServoControl(servo_2, 0.025);
                    // When Button 2 is pressed move to state-2
                    if(button_2.isPressed() == true)
                        CurrentState = States.STATE_2;
                    break;
                case STATE_2:
                    //Move Servo-1 CM until hit button 1,
                    // Servo-2 reset to starting position
                    ServoControl(servo_1, -0.025);
                    servo_2.setPosition(0.0);
                    // When button is pressed move to STATE-3
                    if(button_1.isPressed() == true)
                        CurrentState = States.STATE_3;
                    break;
                case STATE_3:
                    //Move Servo-2 CCM until hit button 2,
                    //Servo-1 reset to starting position
                    ServoControl(servo_2, 0.025);
                    servo_1.setPosition(1.0);
                    // When button is pressed move to STATE-2
                    if(button_2.isPressed() == true)
                        CurrentState = States.STATE_2;
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
