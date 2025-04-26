package org.firstinspires.ftc.teamcode.main;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import org.firstinspires.ftc.teamcode.math.navigation.CoordSys;
import org.firstinspires.ftc.teamcode.math.navigation.navigators.AStarNavigator;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.math.util.Units;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.StartingPose;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.maps.CenterStageObstacleMap;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;
import org.robolectric.shadows.ShadowLog;

import java.util.ArrayList;

@RunWith(RobolectricTestRunner.class)
public class SimpleTrajectoryGenerationTest {
    @Test
    public void testSimpleTrajectoryGeneration() {
//        Pose2d start = CoordSys.fieldToRobot(StartingPose.RED_AUDIENCE.pose);
//        Pose2d end   = CoordSys.fieldToRobot(StartingPose.RED_BACKSTAGE.pose);
//
//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//                new Pose2d(start.getX(), start.getY(), Rotation2d.fromDegrees(180)),
//                new ArrayList<>(),
//                new Pose2d(end.getX(), end.getY(), Rotation2d.fromDegrees(0)),
//                MyRobot.RobotConfig.trajectoryConfig
//        );

//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//                new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
//                new ArrayList<>(),
//                new Pose2d(0, ObstacleMap.INCHES_PER_TILE * 2, Rotation2d.fromDegrees(90)),
//                MyRobot.RobotConfig.trajectoryConfig
//        );
//
//
//        if (trajectory != null) {
//            TrajectorySequence.logTrajectory(trajectory);
//        }
    }
}
