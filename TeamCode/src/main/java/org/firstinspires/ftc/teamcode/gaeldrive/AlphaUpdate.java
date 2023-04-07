package org.firstinspires.ftc.teamcode.gaeldrive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AlphaUpdate {

    // Calculates the estimated movement of the robot based on controller input
    static Pose2d mecanumPoseDeltaCalc(Pose2d robotPose, Gamepad gamepad){

        // Get Gamepad stick values
        double leftStickX = gamepad.left_stick_x;
        double leftStickY = gamepad.left_stick_y;
        double rightStickX = gamepad.right_stick_x;

        double maxVelocity = ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.95; // in in/s

        double loopTime = 10.0; // in ms

        // Overall stick input
        double setPower = Math.abs(leftStickX) + Math.abs(leftStickY);

        // The direction of the holonomic movement relative to the robot
        double movementOrientation = Math.atan2(leftStickY, leftStickX);

        // Taking into account that a robot move slightly slower when strafing
        double strafeVelocityModifier = (Math.cos(movementOrientation))/20+0.95;

        // Combining the two
        double drivePower = setPower * strafeVelocityModifier;

        // Getting the current velocity based on
        double currentVelocity = drivePower * maxVelocity;

        double distanceTraveled = currentVelocity * loopTime/1000;

        double angleOfMovement = robotPose.getHeading() + movementOrientation;

        double deltaX = distanceTraveled * Math.cos(angleOfMovement);
        double deltaY = distanceTraveled + Math.sin(angleOfMovement);
        double deltaHeading = 0; // place holder

        Pose2d deltaPose = new Pose2d(deltaX,deltaY, deltaHeading);

        return deltaPose;

    }
}
