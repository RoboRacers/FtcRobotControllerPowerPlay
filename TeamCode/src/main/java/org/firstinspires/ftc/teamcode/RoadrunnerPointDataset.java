package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

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

    public RoadrunnerPointDataset(SampleMecanumDrive drive, MultipleTelemetry telemetry) {
        lDrive = drive;
        ltelementry = telemetry;
    }

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

    /*
    // Medium Pole Cyle w/ Preload Cone + Parking
    class MediumCycle {
        // Starting Point 0
        class SP0 {
            Trajectory PP1 = lDrive.trajectoryBuilder(new Pose2d(36, -64.5, Math.toRadians(-270)))
                    .forward(29)
                    //.turn(Math.toRadians(45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(-45))
                    //.turn(Math.toRadians(90))
                    .forward(21)
                    //.turn(Math.toRadians(-90))
                    .build();
            Trajectory PP2 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(-45))
                    .build();
            Trajectory PP3 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(-45))
                    //.turn(Math.toRadians(-90))
                    .forward(21)
                    //.turn(Math.toRadians(90))
                    .build();
        }
        // Starting Point 1
        class SP1 {
            Trajectory PP1 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(-45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(45))
                    //.turn(Math.toRadians(90))
                    .forward(21)
                    //.turn(Math.toRadians(-90))
                    .build();
            Trajectory PP2 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(-45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(45))
                    .build();
            Trajectory PP3 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(-45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(45))
                    //.turn(Math.toRadians(-90))
                    .forward(21)
                    //.turn(Math.toRadians(90))
                    .build();
        }
        // Starting Point 2
        class SP2 {
            Trajectory PP1 = lDrive.trajectoryBuilder(new Pose2d(-36, -64.5, Math.toRadians(-270)))
                    .forward(29)
                    //.turn(Math.toRadians(45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(-45))
                    //.turn(Math.toRadians(90))
                    .forward(21)
                    //.turn(Math.toRadians(-90))
                    .build();
            Trajectory PP2 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(-45))
                    .build();
            Trajectory PP3 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(-45))
                    //.turn(Math.toRadians(-90))
                    .forward(21)
                    //.turn(Math.toRadians(90))
                    .build();
        }
        // Starting Point 3
        class SP3 {
            Trajectory PP1 = lDrive.trajectoryBuilder(new Pose2d(-36, -64.5, Math.toRadians(-270)))
                    .forward(29)
                    //.turn(Math.toRadians(-45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(45))
                    //.turn(Math.toRadians(90))
                    .forward(21)
                    //.turn(Math.toRadians(-90))
                    .build();
            Trajectory PP2 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(-45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(45))
                    .build();
            Trajectory PP3 = lDrive.trajectoryBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
                    .forward(29)
                    //.turn(Math.toRadians(-45))
                    .forward(5)
                    //.waitSeconds(1.5)
                    .back(5)
                    //.turn(Math.toRadians(45))
                    //.turn(Math.toRadians(-90))
                    .forward(21)
                    //.turn(Math.toRadians(90))
                    .build();
        }
    }
*/
    // Type, Number, X, Y
    // Ground Junction
    Object[][] Points = {
            {PointType.G, 0, 0, 0},
            {PointType.G, 1, 0, -48},
            {PointType.G, 2, 48, -48},
            {PointType.G, 3, 48, 0},
            {PointType.G, 4, 48, 48},
            {PointType.G, 5, 0, 48},
            {PointType.G, 6, -48, 48},
            {PointType.G, 7, -48, 0},
            {PointType.G, 8, -48, -48},

            // Low Junction
            {PointType.L, 0, 24, -48},
            {PointType.L, 1, 48, -24},
            {PointType.L, 2, 48, 24},
            {PointType.L, 3, 24, 48},
            {PointType.L, 4, -24, 48},
            {PointType.L, 5, -48, 24},
            {PointType.L, 6, -48, -24},
            {PointType.L, 7, -24, -48},

            // Medium Junction
            {PointType.M, 0, 24, -24},
            {PointType.M, 1, 24, 24},
            {PointType.M, 2, -24, 24},
            {PointType.M, 3, -24, -24},

            // High Junction
            {PointType.H, 0, 0, -24},
            {PointType.H, 1, 24, 0},
            {PointType.H, 2, 0, 24},
            {PointType.H, 3, -24, 0},

            // Blue Stack
            {PointType.BS, 0, 72, 12},
            {PointType.BS, 1, -72, 12},

            // Red Stack
            {PointType.RS, 0, 72, -12},
            {PointType.RS, 1, -72, -12},

            // Terminal
            {PointType.T, 0, 60, -60},
            {PointType.T, 1, 60, 60},
            {PointType.T, 2, -60, 60},
            {PointType.T, 3, -60, -60},

            // Signal
            {PointType.SI, 0, -36, -36},
            {PointType.SI, 1, 36, -36},
            {PointType.SI, 2, 36, 36},
            {PointType.SI, 3, -36, 36},

            // Blue Substation
            {PointType.BSUB, 0, 60, 0},

            // Red SUbsation
            {PointType.RSUB, 0, -60, 0},

            // 64.5, 36 -> 36, -64.5

            // Starting Points
            // Subject to change based on the robot size
            //
            {PointType.SP, 0, 36, -64.5, Math.toRadians(270)},
            {PointType.SP, 1, 36, 64.5, Math.toRadians(270)},
            {PointType.SP, 2, -36, 64.5, Math.toRadians(-270)},
            {PointType.SP, 3, -36, -64.5, Math.toRadians(-270)},

            // Parking Postions for SP 0 (Red Right)
            /*
            * drive.trajectorySequenceBuilder(new Pose2d(36, -64.5, Math.toRadians(-270)))
            * #1 .lineTo(new Vector2d(36, -35))
                 .turn(Math.toRadians(90))
                 .lineTo(new Vector2d(12, -35))
                 .turn(Math.toRadians(-90))
                 .lineTo(new Vector2d(12, -24))
            * #2 .lineTo(new Vector2d(36, -24))
            * #3 .lineTo(new Vector2d(36, -35))
                 .turn(Math.toRadians(-90))
                 .lineTo(new Vector2d(57.75, -35))
                 .turn(Math.toRadians(90))
                 .lineTo(new Vector2d(57.75, -24))
            */
            {PointType.PPl0, 1, 12, -24},
            {PointType.PPl0, 2, 36, -24},
            {PointType.PPl0, 3, 60, -24},

            // Parking Postions for SP 1 (Blue Right)
            /*
            * drive.trajectorySequenceBuilder(new Pose2d(36, 64.5, Math.toRadians(270)))
            * #1 .lineTo(new Vector2d(36, 35))
                 .turn(Math.toRadians(90))
                 .lineTo(new Vector2d(58, 35))
                 .turn(Math.toRadians(-90))
                 .lineTo(new Vector2d(58, 24))
            * #2 .lineTo(new Vector2d(36, 24))
            * #3 .lineTo(new Vector2d(36, 36))
                 .turn(Math.toRadians(-90))
                 .lineTo(new Vector2d(12, 36))
                 .turn(Math.toRadians(90))
                 .lineTo(new Vector2d(12, 24))
            */

            {PointType.PPl1, 1, 60, 24},
            {PointType.PPl1, 2, 36, 24},
            {PointType.PPl1, 3, 12, 24},

            // Parking Postions for SP 2 (Blue Left)
            /*
            * drive.trajectorySequenceBuilder(new Pose2d(-36, 64.5, Math.toRadians(270)))
            * #1 .lineTo(new Vector2d(-36, 35))
                 .turn(Math.toRadians(90))
                 .lineTo(new Vector2d(-12, 35))
                 .turn(Math.toRadians(-90))
                 .lineTo(new Vector2d(-12, 24))
            * #2 .lineTo(new Vector2d(-36, 24))
            * #3 .lineTo(new Vector2d(-36, 35))
                 .turn(Math.toRadians(-90))
                 .lineTo(new Vector2d(-57.75, 35))
                 .turn(Math.toRadians(90))
                 .lineTo(new Vector2d(-57.75, 24))
            */
            {PointType.PPl2, 1, -12, 24},
            {PointType.PPl2, 2, -36, 24},
            {PointType.PPl2, 3, -60, 24},

            // Parking Postions for SP 3 (Red Left)
            /*
            * drive.trajectorySequenceBuilder(new Pose2d(-36, -64.5, Math.toRadians(-270)))
            * #1 .lineTo(new Vector2d(-36, -36))
                 .turn(Math.toRadians(-90))
                 .lineTo(new Vector2d(-12, -36))
                 .turn(Math.toRadians(90))
                 .lineTo(new Vector2d(-12, -24))
            * #2 .lineTo(new Vector2d(-36, -24))
            * #3 .lineTo(new Vector2d(-36, -36))
                 .turn(Math.toRadians(-90))
                 .lineTo(new Vector2d(-12, -36))
                 .turn(Math.toRadians(90))
                 .lineTo(new Vector2d(-12, -24))
            */
            {PointType.PPl3, 1, -60, -24},
            {PointType.PPl3, 2, -36, -24},
            {PointType.PPl3, 3, -12, -24}


            /*
            * Cycle w/ Medium Pole Preload (SP1,3)
            * .forward(29)
                 .turn(Math.toRadians(-45))
                 .forward(5)
                 .waitSeconds(1.5)
                 .back(5)
                 .turn(Math.toRadians(45))
            * Cycle w/ Medium Pole Preload (SP0, 2)
            * .forward(29)
                 .turn(Math.toRadians(45))
                 .forward(5)
                 .waitSeconds(1.5)
                 .back(5)
                 .turn(Math.toRadians(-45))
            * Parking After
            * #1 .turn(Math.toRadians(90))
                 .forward(21)
                 .turn(Math.toRadians(-90))
            * #2 nothing
            * #3 .turn(Math.toRadians(-90))
                 .forward(21)
                 .turn(Math.toRadians(90))
            */
    };

}
