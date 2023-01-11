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
import org.firstinspires.ftc.teamcode.AutoopStateMachinesLeft;
import org.firstinspires.ftc.teamcode.AutoopStateMachinesRight;
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
    final int stack1 = -225;
    final int liftHigherThanLow = -600;
    final int liftMid = -900;
    final int liftHigh = -1250;

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
    TrajectorySequence traj0;

    Trajectory traj1;
    Trajectory traj2;
    TrajectorySequence traj3;
    Trajectory traj4;
    TrajectorySequence traj5;
    Trajectory traj6;
    TrajectorySequence trajSeq6;
    TrajectorySequence traj7;
    Trajectory traj8;

    public void MediumCycleRight () {
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
                    ArmPosition(liftMid, 1.0);
                })
                .build();
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(-24, 32))
                .addSpatialMarker(new Vector2d(-24, 32),() -> {
                    lclaw.setPosition(open);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-24, 36))
                .addSpatialMarker(new Vector2d(-24, 36),() -> {
                    ArmPosition(liftLow,1.0);
                })
                .build();
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);
        lDrive.update();
    }

    public void HighPreloadRight () {
        Pose2d StartPose = new Pose2d(-36, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Preload
        traj1 = lDrive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(-36, 12))
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-24, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(-24, 9))
                .waitSeconds(5)
                .addTemporalMarker(7, () -> {
                    claw(open);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-24, 16))
                .addSpatialMarker(new Vector2d(-24, 16),() -> {
                    ArmPosition(liftLow, 1);
                })
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);

        lDrive.update();
    }

    public void HighPreloadRightV1 () {
        Pose2d StartPose = new Pose2d(-33, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Shift
        traj0 = lDrive.trajectorySequenceBuilder(StartPose)
                .strafeTo(new Vector2d(-36, 64.5))
                .build();

        // Preload
        traj1 = lDrive.trajectoryBuilder(traj0.end())
                .lineTo(new Vector2d(-36, 12))
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-25, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(-25, 9))
                .waitSeconds(3)
                .addTemporalMarker(5, () -> {
                    claw(open);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-25, 16))
                .addSpatialMarker(new Vector2d(-24, 16),() -> {
                    ArmPosition(liftLow, 1);
                })
                .build();

        lDrive.followTrajectorySequence(traj0);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);

        lDrive.update();
    }

    public void HighPreloadRightV2 () {
        Pose2d StartPose = new Pose2d(-33, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Shift
        traj0 = lDrive.trajectorySequenceBuilder(StartPose)
                .strafeTo(new Vector2d(-36, 64.5))
                .build();

        // Preload
        traj1 = lDrive.trajectoryBuilder(traj0.end())
                .lineTo(new Vector2d(-36, 12))
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(-36, 20),() -> {
                    ArmPosition(stack1+50, 1);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-26, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(-26, 6))
                .waitSeconds(2)
                .addTemporalMarker(3, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(4, () -> {
                    claw(open);
                })
                .waitSeconds(2)
                .addTemporalMarker(5, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-26, 16))
                .addSpatialMarker(new Vector2d(-24, 16),() -> {
                    ArmPosition(liftLow, 1);
                })
                .build();

        lDrive.followTrajectorySequence(traj0);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);

        lDrive.update();
    }

    public void HighCycleRightV1 () {
        Pose2d StartPose = new Pose2d(-33, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Shift
        traj0 = lDrive.trajectorySequenceBuilder(StartPose)
                .strafeTo(new Vector2d(-36, 64.5))
                .build();

        // Preload
        traj1 = lDrive.trajectoryBuilder(traj0.end())
                .lineTo(new Vector2d(-36, 12))
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(-36, 20),() -> {
                    ArmPosition(stack1+200, 1);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-26, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(-26, 9))
                .waitSeconds(1)
                .addTemporalMarker(2, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(3, () -> {
                    claw(open);
                })
                .waitSeconds(1)
                .addTemporalMarker(4, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-26, 16))
                .addSpatialMarker(new Vector2d(-24, 14),() -> {
                    ArmPosition(stack1, 1);
                })
                .build();

        traj5 = lDrive.trajectorySequenceBuilder(traj4.end())
                //.strafeTo(new Vector2d(-54, 12))
                .lineToLinearHeading(new Pose2d(-62, 23.5, Math.toRadians(180)))
                .waitSeconds(4)
                .addTemporalMarker(3, () -> {
                    claw(close);
                })
                .addTemporalMarker(4, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        trajSeq6 = lDrive.trajectorySequenceBuilder(traj5.end())
                //.strafeTo(new Vector2d(-54, 12))
                .lineToLinearHeading(new Pose2d(-24, 14, Math.toRadians(270)))
                .build();
        traj7 = lDrive.trajectorySequenceBuilder(trajSeq6.end())
                .lineTo(new Vector2d(-26, 9))
                .waitSeconds(1)
                .addTemporalMarker(2, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(3, () -> {
                    claw(open);
                })
                .waitSeconds(1)
                .addTemporalMarker(4, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();

        lDrive.followTrajectorySequence(traj0);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);
        lDrive.followTrajectorySequence(traj5);
        lDrive.followTrajectorySequence(trajSeq6);
        lDrive.followTrajectorySequence(traj7);

        lDrive.update();
    }

    public void HighPreloadLeft () {
        Pose2d StartPose = S1_POS;
        lDrive.setPoseEstimate(StartPose);

        // Preload
        traj1 = lDrive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(36, 12))
                .addSpatialMarker(new Vector2d(36, 45),() -> {
                    claw(close);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(24, 12))
                .addSpatialMarker(new Vector2d(24, 12),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(24, 7))
                .waitSeconds(5)
                .addTemporalMarker(7, () -> {
                    claw(open);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(24, 14))
                .addSpatialMarker(new Vector2d(24, 14),() -> {
                    ArmPosition(liftLow, 1);
                })
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);

        lDrive.update();
    }

    public void HighPreloadLeftV2 () {
        Pose2d StartPose = new Pose2d(38, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Shift
        traj0 = lDrive.trajectorySequenceBuilder(StartPose)
                .strafeTo(new Vector2d(34.5, 64.5))
                .build();

        // Preload
        traj1 = lDrive.trajectoryBuilder(traj0.end())
                .lineTo(new Vector2d(34.5, 12))
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(36, 20),() -> {
                    ArmPosition(stack1+100, 1);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(19, 12))
                .addSpatialMarker(new Vector2d(24, 12),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(19, 6))
                .waitSeconds(2)
                .addTemporalMarker(3, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(4, () -> {
                    claw(open);
                })
                .waitSeconds(2)
                .addTemporalMarker(5, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(19, 16))
                .addSpatialMarker(new Vector2d(24, 16),() -> {
                    ArmPosition(liftLow, 1);
                })
                .build();

        lDrive.followTrajectorySequence(traj0);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);

        lDrive.update();
    }

    /* Parking after Preload */

    public void PreloadParkingRightPP1 () {
        lDrive.setPoseEstimate(new Pose2d(-24, 16, Math.toRadians(270)));
        Trajectory parkingtraj1 = lDrive.trajectoryBuilder(new Pose2d(-24, 16, Math.toRadians(270)))
                .strafeTo(new Vector2d(-12, 16))
                .build();
        lDrive.followTrajectory(parkingtraj1);
        lDrive.update();
    }

    public void PreloadParkingRightPP2 () {
        lDrive.setPoseEstimate(new Pose2d(-24, 16, Math.toRadians(270)));
        Trajectory parkingtraj2 = lDrive.trajectoryBuilder(new Pose2d(-24, 16, Math.toRadians(270)))
                .strafeTo(new Vector2d(-36, 16))
                .build();
        lDrive.followTrajectory(parkingtraj2);
        lDrive.update();
    }

    public void PreloadParkingRightPP3 () {
        lDrive.setPoseEstimate(new Pose2d(-24, 16, Math.toRadians(270)));
        Trajectory parkingtraj3 = lDrive.trajectoryBuilder(new Pose2d(-24, 16, Math.toRadians(270)))
                .strafeTo(new Vector2d(-60, 16))
                .build();
        lDrive.followTrajectory(parkingtraj3);
        lDrive.update();
    }

    public void PreloadParkingLeftPP1 () {
        lDrive.setPoseEstimate(new Pose2d(24, 14, Math.toRadians(270)));
        Trajectory parkingtraj1 = lDrive.trajectoryBuilder(new Pose2d(24, 14, Math.toRadians(270)))
                .strafeTo(new Vector2d(60, 14))
                .build();
        lDrive.followTrajectory(parkingtraj1);
        lDrive.update();
    }

    public void PreloadParkingLeftPP2 () {
        lDrive.setPoseEstimate(new Pose2d(24, 14, Math.toRadians(270)));
        Trajectory parkingtraj2 = lDrive.trajectoryBuilder(new Pose2d(24, 14, Math.toRadians(270)))
                .strafeTo(new Vector2d(36, 14))
                .build();
        lDrive.followTrajectory(parkingtraj2);
        lDrive.update();
    }

    public void PreloadParkingLeftPP3 () {
        lDrive.setPoseEstimate(new Pose2d(24, 14, Math.toRadians(270)));
        Trajectory parkingtraj3 = lDrive.trajectoryBuilder(new Pose2d(24, 14, Math.toRadians(270)))
                .strafeTo(new Vector2d(12, 14))
                .build();
        lDrive.followTrajectory(parkingtraj3);
        lDrive.update();
    }

    /* Testing Trajectories */

    public void LineToLHTest () {
        Pose2d StartPose = new Pose2d(-24, 14, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);


        traj5 = lDrive.trajectorySequenceBuilder(StartPose)
                //.strafeTo(new Vector2d(-54, 12))
                .lineToLinearHeading(new Pose2d(-60, 20, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(-60, 20),() -> {
                    //ArmPosition(stack1);
                })
                .waitSeconds(2)
                .build();
        traj6 = lDrive.trajectoryBuilder(traj5.end())
                //.strafeTo(new Vector2d(-54, 12))
                .lineToLinearHeading(new Pose2d(-24, 14, Math.toRadians(270)))
                .addSpatialMarker(new Vector2d(-24, 14),() -> {
                    //ArmPosition(high);
                })
                .build();



        lDrive.followTrajectorySequence(traj5);
        lDrive.followTrajectory(traj6);

        lDrive.update();
    }

    public void AlignmentTest () {
        Pose2d StartPose = new Pose2d(38, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        traj1 = lDrive.trajectoryBuilder(StartPose)
                .lineTo(new Vector2d(14, 64.5))
                .build();

        lDrive.followTrajectory(traj1);

        lDrive.update();
    }

    public void HighPreloadRightTest () {
        Pose2d StartPose = new Pose2d(-38, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        traj1 = lDrive.trajectoryBuilder(StartPose)
                .strafeTo(new Vector2d(-36, 64.5))
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-36, 12))
                .build();
        traj4 = lDrive.trajectoryBuilder(traj2.end())
                .strafeTo(new Vector2d(-24, 12))
                .build();
        traj6 = lDrive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(-24, 9))
                .build();
        traj8 = lDrive.trajectoryBuilder(traj6.end())
                .lineTo(new Vector2d(-24, 16))
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectory(traj4);
        lDrive.followTrajectory(traj6);
        lDrive.followTrajectory(traj8);

        lDrive.update();
    }

    // Functions

    public void ArmPosition(int pos, double speed) {
        lmotorRight.setTargetPosition(-pos);
        lmotorLeft.setTargetPosition(-pos);
        lmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotorLeft.setPower(speed);
        lmotorRight.setPower(speed);
    }

    public void claw(double pos) {
        lclaw.setPosition(pos);
    }
}
