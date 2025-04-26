package org.firstinspires.ftc.teamcode.robotcore.game.element;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.StartingPose;

import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Prop {
    public enum PropPosition {
        LEFT(Stream.of(new Object[][] {
                { StartingPose.RED_AUDIENCE, StartingPose.RED_AUDIENCE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.15, -3),
                                new Rotation2d(-20)
                        )
                ) },
                { StartingPose.RED_BACKSTAGE, StartingPose.RED_BACKSTAGE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.15, -4),
                                new Rotation2d(-30)
                        )
                ) },
                { StartingPose.BLUE_AUDIENCE, StartingPose.BLUE_AUDIENCE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.15, -4),
                                new Rotation2d(-30)
                        )
                ) },
                { StartingPose.BLUE_BACKSTAGE, StartingPose.BLUE_BACKSTAGE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.15, -3),
                                new Rotation2d(-20)
                        )
                ) },
        }).collect(Collectors.toMap(data -> (StartingPose) data[0], data -> (Pose2d) data[1]))),
        CENTER(Stream.of(new Object[][] {
                { StartingPose.RED_AUDIENCE, StartingPose.RED_AUDIENCE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.25, 3),
                                new Rotation2d(0)
                        )
                ) },
                { StartingPose.RED_BACKSTAGE, StartingPose.RED_BACKSTAGE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.2, 3),
                                new Rotation2d(0)
                        )
                ) },
                { StartingPose.BLUE_AUDIENCE, StartingPose.BLUE_AUDIENCE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.2, 3),
                                new Rotation2d(0)
                        )
                ) },
                { StartingPose.BLUE_BACKSTAGE, StartingPose.BLUE_BACKSTAGE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.25, 3),
                                new Rotation2d(0)
                        )
                ) },
        }).collect(Collectors.toMap(data -> (StartingPose) data[0], data -> (Pose2d) data[1]))),
        RIGHT(Stream.of(new Object[][] {
                { StartingPose.RED_AUDIENCE, StartingPose.RED_AUDIENCE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.1, ObstacleMap.INCHES_PER_TILE * 0.51),
                                new Rotation2d(45)
                        )
                ) },
                { StartingPose.RED_BACKSTAGE, StartingPose.RED_BACKSTAGE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.1, ObstacleMap.INCHES_PER_TILE * 0.4),
                                new Rotation2d(45)
                        )
                ) },
                { StartingPose.BLUE_AUDIENCE, StartingPose.BLUE_AUDIENCE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.1, ObstacleMap.INCHES_PER_TILE * 0.4),
                                new Rotation2d(45)
                        )
                ) },
                { StartingPose.BLUE_BACKSTAGE, StartingPose.BLUE_BACKSTAGE.pose.transformBy(
                        new Transform2d(
                                new Translation2d(ObstacleMap.INCHES_PER_TILE * 1.1, ObstacleMap.INCHES_PER_TILE * 0.51),
                                new Rotation2d(45)
                        )
                ) },
        }).collect(Collectors.toMap(data -> (StartingPose) data[0], data -> (Pose2d) data[1]))),
        NONE(null);
        public final Map<StartingPose, Pose2d> fieldPositions;
        PropPosition(Map<StartingPose, Pose2d> fieldPositions) {
            this.fieldPositions = fieldPositions;
        }
    }
}
