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
    final int stack1 = -275;
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
    TrajectorySequence traj7;
    Trajectory traj8;
    Trajectory traj9, traj10, traj11;

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

    public void HighCycleRight () {
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
                .lineTo(new Vector2d(-24, 10))
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

        /// Cycle
        traj5 = lDrive.trajectorySequenceBuilder(traj4.end())
                //.strafeTo(new Vector2d(-54, 12))
                .lineToLinearHeading(new Pose2d(-60, 20, Math.toRadians(180)))
                .addSpatialMarker(new Vector2d(-60, 20),() -> {
                    ArmPosition(stack1, 1.0);
                })
                .waitSeconds(6)
                .addTemporalMarker(6, () -> {
                    claw(close);
                })
                .build();
        traj6 = lDrive.trajectoryBuilder(traj5.end())
                .lineToLinearHeading(new Pose2d(-24, 16, Math.toRadians(270)))
                .addTemporalMarker(0, () -> {
                    ArmPosition(liftHigh, 0.3);
                })
                .build();

        traj7 = lDrive.trajectorySequenceBuilder(traj6.end())
                .lineTo(new Vector2d(-24, 10))
                .addSpatialMarker(new Vector2d(-24, 10),() -> {
                    claw(open);
                })
                .build();

        lDrive.followTrajectory(traj1);
        lDrive.followTrajectory(traj2);
        lDrive.followTrajectorySequence(traj3);
        lDrive.followTrajectory(traj4);

        // lDrive.followTrajectorySequence(traj5);
        // lDrive.followTrajectory(traj6);
        /*
        lDrive.followTrajectorySequence(traj7);
         */

        lDrive.update();
    }

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

    public void HighCycleRightSingle () {
        Pose2d StartPose = new Pose2d(-36, 64.5, Math.toRadians(270));
        lDrive.setPoseEstimate(StartPose);

        traj0 = lDrive.trajectorySequenceBuilder(StartPose)

                // Move forward
                .lineTo(new Vector2d(-36, 12))
                .addSpatialMarker(new Vector2d(-36, 12),() -> {
                    claw(close);
                })

                // Strafe to pole, raise lift
                .strafeTo(new Vector2d(-24, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftHigh, 1.0);
                })

                // Move forward a bit, then open claw
                .lineTo(new Vector2d(-24, 10))
                .addSpatialMarker(new Vector2d(-24, 10),() -> {
                    claw(open);
                })

                // Move backwards a bit and lower lift
                /*
                .lineTo(new Vector2d(-24, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftLow);
                })
                */

                /*.strafeTo(new Vector2d(-54, 12))
                .lineToLinearHeading(new Pose2d(-54, 12, Math.toRadians(90)))
                .addSpatialMarker(new Vector2d(-54, 12),() -> {
                    ArmPosition(stack1);
                })
                //.turn(Math.toRadians(-90))

                 */

                /*
                .lineTo(new Vector2d(-61, 12))
                .addSpatialMarker(new Vector2d(-60, 12),() -> {
                    claw(close);
                })

                .lineTo(new Vector2d(-56, 12))
                .turn(Math.toRadians(90))

                .strafeTo(new Vector2d(-24, 12))
                .addSpatialMarker(new Vector2d(-24, 12),() -> {
                    ArmPosition(liftHigh);
                })

                .lineTo(new Vector2d(-24, 10))
                .addSpatialMarker(new Vector2d(-24, 10),() -> {
                    claw(open);
                })
                */

                .build();

        lDrive.followTrajectorySequence(traj0);

        lDrive.update();
    }

    public void ArmPosition(int pos, double speed) {
        lmotorLeft.setPower(0);
        lmotorRight.setPower(0);
        lmotorRight.setTargetPosition(pos);
        lmotorLeft.setTargetPosition(pos);
        lmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lmotorLeft.setPower(speed);
        lmotorRight.setPower(speed);
    }

    public void claw(double pos) {
        lclaw.setPosition(pos);
    }
}
