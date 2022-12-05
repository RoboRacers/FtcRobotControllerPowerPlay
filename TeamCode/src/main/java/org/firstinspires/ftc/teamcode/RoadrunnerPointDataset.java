package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.AutoopStateMachines;
import java.io.*;
import java.util.*;

public class RoadrunnerPointDataset {

    /*
     * This contains all the points for the field elements for roadrunner
     */
    enum PointType {
        G, // Ground
        L, // Low
        M, // Medium
        H, // High
        BS, // Blue Stack
        RS, // Red sTack
        T, // Terminal
        SI, // Signal
        BSUB, // Blue Substation
        RSUB, // Red Substation
        SP, // Starting Point
        PPl0, // Parking Position for SP0
        PPl1, // Parking Position for SP1
        PPl2, // Parking Position for SP2
        PPl3 // Parking Position for SP3
    }


    private SampleMecanumDrive lDrive;
    private MultipleTelemetry ltelementry;

    public final Pose2d S0_POS = new Pose2d(36, -64.5, Math.toRadians(-270));
    public final Pose2d S1_POS = new Pose2d(36, 64.5, Math.toRadians(270));
    public final Pose2d S2_POS = new Pose2d(-36, 64.5, Math.toRadians(270));
    public final Pose2d S3_POS = new Pose2d(-36, -64.5, Math.toRadians(-270));

    final int liftLow = 0;
    final int stack1 = -350;
    final int liftHigherThanLow = -600;
    final int liftMid = -900;
    final int liftHigh = -1200;

    DcMotorEx lmotorLeft;
    DcMotorEx lmotorRight;
    Servo lclaw;

    final double close = 0.7;
    final double open =0;

    public RoadrunnerPointDataset(SampleMecanumDrive drive, MultipleTelemetry telemetry, DcMotorEx motorRight, DcMotorEx motorLeft, Servo claw) {
        lDrive = drive;
        ltelementry = telemetry;

        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lmotorLeft = motorLeft;
        lmotorRight = motorRight;

        lclaw = claw;
    }



    AutoopStateMachines AutoCaller;

    public void test() {
        Pose2d StartPose = new Pose2d(36, -64.5, Math.toRadians(-270));
        lDrive.setPoseEstimate(StartPose);

        Trajectory traj1 = lDrive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(36, -35))
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(12, -35))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
    }

    public void S0PP1 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S0_POS)
                .lineTo(new Vector2d(36, -35))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(12, -35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(12, -24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S0PP2 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S0_POS)
                .lineTo(new Vector2d(36, -24))
                .build();
        lDrive.followTrajectory(traj1);
    }

    public void S0PP3 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S0_POS)
                .lineTo(new Vector2d(36, -35))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(57.75, -35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(57.75, -24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S1PP1 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S1_POS)
                .lineTo(new Vector2d(36, 35))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(58, 35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(58, 24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S1PP2 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S1_POS)
                .lineTo(new Vector2d(36, 24))
                .build();
        lDrive.followTrajectory(traj1);
    }

    public void S1PP3 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S1_POS)
                .lineTo(new Vector2d(36, 36))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(12, 36))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(12, 24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
        }

    public void S2PP1 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S2_POS)
                .lineTo(new Vector2d(-36, 35))
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-12, 35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-12, 24))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S2PP2 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S2_POS)
                .lineTo(new Vector2d(-36, 24))
                .build();
        lDrive.followTrajectory(traj1);
    }

    public void S2PP3 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S2_POS)
                .lineTo(new Vector2d(-36, 35))
                .build();

        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-57.75, 35))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-57.75, 24))
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S3PP1 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S3_POS)
                .lineTo(new Vector2d(-36, -36))
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-57.75, -36))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-57.75, -24))
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    public void S3PP2 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S3_POS)
                .lineTo(new Vector2d(-36, -24))
                .build();
        lDrive.followTrajectory(traj1);
    }

    public void S3PP3 () {
        Trajectory traj1 = lDrive.trajectoryBuilder(S3_POS)
                .lineTo(new Vector2d(-36, -36))
                .build();
        Trajectory traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-12, -36))
                .build();
        Trajectory traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-12, -24))
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
    }

    // Medium Cycles Trajectories Init
    Trajectory traj1;
    Trajectory traj2;
    Trajectory traj3;
    Trajectory traj4;
    Trajectory traj5;
    Trajectory traj6;
    Trajectory traj7;
    Trajectory traj8;
    Trajectory traj9, traj10, traj11;

    public void S2PP0MediumCycle1 () {
        traj1 = lDrive.trajectoryBuilder(S2_POS)
                .lineTo(new Vector2d(-36, 36))
                .addDisplacementMarker(() -> {
                    lDrive.followTrajectory(traj2);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-24, 36))
                .addDisplacementMarker(() -> {
                    ArmPosition(liftMid);

                })
                .build();

        lDrive.followTrajectory(traj1);
    }

    public void S2PP0MediumCycle2 () {
        Pose2d StartPose = new Pose2d(-36, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);
        traj1 = lDrive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(-36, 36))
                .addSpatialMarker(new Vector2d(-36, 36),() -> {
                    claw(close);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-24, 36))
                .addSpatialMarker(new Vector2d(-24, 36),() -> {
                    ArmPosition(liftMid);
                })
                .build();
        traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-24, 32))
                .addSpatialMarker(new Vector2d(-24, 32),() -> {
                    lclaw.setPosition(open);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-24, 36))
                .addSpatialMarker(new Vector2d(-24, 36),() -> {
                    ArmPosition(liftLow);
                })
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
        lDrive.followTrajectory(traj4);
        lDrive.update();
    }

    public void S2PP0HighCycle () {
        Pose2d StartPose = new Pose2d(-36, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);
        traj1 = lDrive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(-36, 12))
                .addSpatialMarker(new Vector2d(-36, 12),() -> {
                    claw(close);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-24, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftHigh);
                })
                .build();
        traj3 = lDrive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-24, 10))
                .addSpatialMarker(new Vector2d(-24, 10),() -> {
                    claw(open);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-24, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftLow);
                })
                .build();

        traj5 = lDrive.trajectoryBuilder(traj4.end())
                .strafeTo(new Vector2d(-54, 12))
                .addSpatialMarker(new Vector2d(-54, 12),() -> {
                    ArmPosition(stack1);
                })
                .build();
        traj6 = lDrive.trajectoryBuilder(traj5.end())
                .lineTo(new Vector2d(-61, 12))
                .addSpatialMarker(new Vector2d(-60, 12),() -> {
                    claw(close);
                })
                .build();
        traj7 = lDrive.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(-56, 12))
                .build();
        traj8 = lDrive.trajectoryBuilder(traj7.end())
                .strafeTo(new Vector2d(-24, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftHigh);
                })
                .build();
        traj9 = lDrive.trajectoryBuilder(traj8.end())
                .lineTo(new Vector2d(-24, 10))
                .addSpatialMarker(new Vector2d(-24, 10),() -> {
                    claw(open);
                })
                .build();


        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj3);
        lDrive.followTrajectory(traj4);
        lDrive.followTrajectory(traj5);
        lDrive.turn(Math.toRadians(-90));
        lDrive.followTrajectory(traj6);
        lDrive.followTrajectory(traj7);
        lDrive.turn(Math.toRadians(90));
        lDrive.followTrajectory(traj8);
        lDrive.followTrajectory(traj9);

        lDrive.update();
    }

    public void ArmPosition(int pos) {
        lmotorLeft.setPower(0);
        lmotorRight.setPower(0);
        lmotorRight.setTargetPosition(pos);
        lmotorLeft.setTargetPosition(pos);
        lmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotorLeft.setPower(1);
        lmotorRight.setPower(1);
    }

    public void claw(double pos) {
        lclaw.setPosition(pos);
    }
}
