package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.AutoopStateMachines;
import org.firstinspires.ftc.teamcode.AutoopStateMachinesLeft;
import org.firstinspires.ftc.teamcode.AutoopStateMachinesRight;
import java.io.*;
import java.util.*;

public class RoadrunnerPointDataset {

    private SampleMecanumDrive lDrive;
    private MultipleTelemetry ltelementry;

    public final Pose2d S0_POS = new Pose2d(36, -64.5, Math.toRadians(-270));
    public final Pose2d S1_POS = new Pose2d(36, 64.5, Math.toRadians(270));
    public final Pose2d S2_POS = new Pose2d(-36, 64.5, Math.toRadians(270));
    public final Pose2d S3_POS = new Pose2d(-36, -64.5, Math.toRadians(-270));

    final int liftLow = 0;
    int stack1 = -285;
    final int liftHigherThanLow = -600;
    final int liftMid = -900;
    final int liftHigh = -1275;

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
    TrajectorySequence trajSeq9;
    TrajectorySequence trajSeq6;
    TrajectorySequence traj7;
    Trajectory traj8;
    TrajectorySequence traj10;

    double preloadXmodifier = -1.5;
    int preloadYmodifier = 1;

    int stackXmodifier = 0;
    int stackYmodifier = -2;

    double cycleXmodifier = -1.5;
    int cycleYmodifier = 1;

    int cycleNumber = 0;



    public void HighPreloadRightV2 () {
        Pose2d StartPose = new Pose2d(-33, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        // Starting Shift
        traj0 = lDrive.trajectorySequenceBuilder(StartPose)
                .strafeTo(new Vector2d(-36, 64.5))
                .build();

        // Preload
        traj1 = lDrive.trajectoryBuilder(traj0.end())
                .lineTo(new Vector2d(-36, 15),
                        lDrive.getVelocityConstraint(85, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        lDrive.getAccelerationConstraint(50)
                )
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(-36, 20),() -> {
                    ArmPosition(-170, 1);
                })
                .build();

        // Shift to Pole
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-25, 15))
                .addSpatialMarker(new Vector2d(-24, 15),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();

        // Drop-Off
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())

                .lineTo(new Vector2d(-25+preloadXmodifier, 11+preloadYmodifier),
                        lDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        lDrive.getAccelerationConstraint(25)
                        )
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(1.5, () -> {
                    claw(open);
                })
                .addTemporalMarker(1.75, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();


        // Backing up
        traj4 = lDrive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-24+preloadYmodifier, 19))
                .addSpatialMarker(new Vector2d(-24+preloadXmodifier, 17),() -> {
                    ArmPosition(stack1, 1);
                    claw(open);
                })
                .build();

        // Go to stack
        traj5 = lDrive.trajectorySequenceBuilder(traj4.end())
                .lineToLinearHeading(new Pose2d(-60+stackXmodifier, 23.5+stackYmodifier, Math.toRadians(180))
                )
                .waitSeconds(0.5)
                .addTemporalMarker(0, () -> {
                    claw(open);
                })
                .addTemporalMarker(2, () -> {
                    claw(close);
                })
                .addTemporalMarker(2.5, () -> {
                    ArmPosition(-500, 0.8);
                })
                .build();

        traj10 = lDrive.trajectorySequenceBuilder(traj5.end())
                .strafeTo(new Vector2d(-60+stackXmodifier, 23.5+stackYmodifier+2))
                .build();

        // Go to pole
        trajSeq6 = lDrive.trajectorySequenceBuilder(traj10.end())
                .lineToLinearHeading(new Pose2d(-24, 20, Math.toRadians(270)))
                .addSpatialMarker(new Vector2d(-50, 20),() -> {
                    ArmPosition(liftHigh, 0.75);
                })
                .build();


        // Drop off Cone
        traj7 = lDrive.trajectorySequenceBuilder(trajSeq6.end())
                .lineTo(new Vector2d(-24+cycleXmodifier, 11+cycleYmodifier),
                        lDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        lDrive.getAccelerationConstraint(25)
                )
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    ArmPosition(liftHigh+350, 1);
                })
                .addTemporalMarker(1.5, () -> {
                    claw(open);
                })
                .addTemporalMarker(1.75, () -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();

        traj8 = lDrive.trajectoryBuilder(traj7.end())
                .lineTo(new Vector2d(-24+cycleXmodifier, 19))
                .addSpatialMarker(new Vector2d(-24+cycleXmodifier, 17),() -> {
                    if (cycleNumber == 0){
                        ArmPosition(stack1, 1);
                    } else if (cycleNumber == 1){
                        ArmPosition(liftLow, 1);
                    }

                })
                .build();

        lDrive.followTrajectorySequence(traj0);
        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);

        lDrive.followTrajectorySequence(traj5);
        lDrive.followTrajectorySequence(traj10);
        lDrive.followTrajectorySequence(trajSeq6);
        lDrive.followTrajectorySequence(traj7);
        lDrive.followTrajectory(traj8);

        stack1 = stack1 + 30;
        cycleNumber = cycleNumber + 1;

        lDrive.followTrajectorySequence(traj5);
        lDrive.followTrajectorySequence(traj10);
        lDrive.followTrajectorySequence(trajSeq6);
        lDrive.followTrajectorySequence(traj7);
        lDrive.followTrajectory(traj8);

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
                .lineTo(new Vector2d(-36, 15))
                .addSpatialMarker(new Vector2d(-36, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(-36, 20),() -> {
                    ArmPosition(stack1+200, 1);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(-26, 15))
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
                .lineTo(new Vector2d(-26, 15))
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
                .addSpatialMarker(new Vector2d(34.5, 45),() -> {
                    claw(close);
                })
                .addSpatialMarker(new Vector2d(34.5, 20),() -> {
                    ArmPosition(stack1+100, 1);
                })
                .build();
        traj2 = lDrive.trajectoryBuilder(traj1.end())
                .strafeTo(new Vector2d(19.5, 12))
                .addSpatialMarker(new Vector2d(24, 12),() -> {
                    ArmPosition(liftHigh, 1);
                })
                .build();
        traj3 = lDrive.trajectorySequenceBuilder(traj2.end())
                .lineTo(new Vector2d(19.5, 10.5))
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
                .lineTo(new Vector2d(19.5, 16))
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
