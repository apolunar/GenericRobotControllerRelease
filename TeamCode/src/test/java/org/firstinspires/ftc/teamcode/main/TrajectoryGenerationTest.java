package org.firstinspires.ftc.teamcode.main;

import android.util.Log;
import android.util.Pair;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySegment;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;
import org.robolectric.shadows.ShadowLog;

@RunWith(RobolectricTestRunner.class)
public class TrajectoryGenerationTest {
    public static int VEL = 50;
    public static int ACCEL = 500;

    @Test
    public void testGenerateTrajectory() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(VEL, ACCEL);
        trajectoryConfig.setStartVelocity(0);
        trajectoryConfig.setEndVelocity(0);

        TrajectorySequence trajectorySequence = TrajectorySequence.weaveTrajectorySequence(
                // TODO: Do not do this! It breaks things because it doesn't actually tell ChassisSubsystem that this is the starting pose
                // It should be changed so it's clearer what this is supposed to do!
//                new Translation2d(0, ObstacleMap.INCHES_PER_TILE*2),
                new TrajectorySegment[]{
//                        new TrajectorySegment( // drive in front of submersible
//                                Rotation2d.fromDegrees(0),
//                                new Translation2d[0],
//                                new Pose2d(ObstacleMap.INCHES_PER_TILE, -20, Rotation2d.fromDegrees(0)),
//                                Rotation2d.fromDegrees(0),
//                                MyRobot.RobotConfig.trajectoryConfig
//                        ),
//                        new TrajectorySegment( // drive forward
//                                Rotation2d.fromDegrees(0),
//                                new Translation2d[0],
//                                new Pose2d(ObstacleMap.INCHES_PER_TILE + 13, -20, Rotation2d.fromDegrees(0)),
//                                Rotation2d.fromDegrees(0),
//                                MyRobot.RobotConfig.trajectoryConfig
//                        ),
//                        new TrajectorySegment(
//                                Rotation2d.fromDegrees(0),
//                                new Translation2d[0],
//                                new Pose2d(ObstacleMap.INCHES_PER_TILE + 13, -10, Rotation2d.fromDegrees(0)),
//                                Rotation2d.fromDegrees(0),
//                                MyRobot.RobotConfig.trajectoryConfig
//                        ),
//                        new TrajectorySegment( // reset
//                                Rotation2d.fromDegrees(-180),
//                                new Translation2d[] {
//                                        new Translation2d(ObstacleMap.INCHES_PER_TILE - 13, 18),
//                                        new Translation2d(ObstacleMap.INCHES_PER_TILE + 10, ObstacleMap.INCHES_PER_TILE),
//                                },
//                                new Pose2d(ObstacleMap.INCHES_PER_TILE * 2, ObstacleMap.INCHES_PER_TILE, Rotation2d.fromDegrees(-180)),
//                                Rotation2d.fromDegrees(0),
//                                MyRobot.RobotConfig.trajectoryConfig
//                        ),
                }
        );

        // ---
        for (Pair<Trajectory, Rotation2d> trajectory : trajectorySequence) {
            for (Trajectory.State state : trajectory.first.getStates()) {
                RobotLog.dd("TRAJECTORY", "State : %s", state);
            }
            RobotLog.dd("TRAJECTORY", "Target rotation : %s deg", trajectory.second.getDegrees());
            RobotLog.dd("TRAJECTORY", "Runtime         : %s s", trajectory.first.getTotalTimeSeconds());
        }

        double totalRuntime = 0;
        for (Pair<Trajectory, Rotation2d> trajectory : trajectorySequence) {
            RobotLog.dd("TRAJECTORY", "Summary         : %s", trajectory.first.getInitialPose());
            RobotLog.dd("TRAJECTORY", "Target rotation : %s deg", trajectory.second.getDegrees());
            RobotLog.dd("TRAJECTORY", "Runtime         : %s s", trajectory.first.getTotalTimeSeconds());
            totalRuntime += trajectory.first.getTotalTimeSeconds();
        }

        RobotLog.dd("TRAJECTORY", "Total runtime : %s s", totalRuntime);

        // ---
    }
}
