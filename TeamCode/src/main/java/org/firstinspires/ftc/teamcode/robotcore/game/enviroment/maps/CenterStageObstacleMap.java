package org.firstinspires.ftc.teamcode.robotcore.game.enviroment.maps;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle.Obstacle;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle.RectangleObstacle;

import java.util.ArrayList;

public class CenterStageObstacleMap implements ObstacleMap {
    @Override
    public Obstacle[] getObstacles() { // TODO: This should be static/final, this does not change just to be optimal
        return new Obstacle[] {
                // backdrop
                new RectangleObstacle(
                        24, 12,
                        new Translation2d(-1.5 * ObstacleMap.INCHES_PER_TILE, 2.75 * ObstacleMap.INCHES_PER_TILE)
                ),
                new RectangleObstacle(
                        24, 12,
                        new Translation2d(1.5 * ObstacleMap.INCHES_PER_TILE, 2.75 * ObstacleMap.INCHES_PER_TILE)
                ),
                // truss legs
                new RectangleObstacle(
                        3, 1. * ObstacleMap.INCHES_PER_TILE,
                        new Translation2d(-2*ObstacleMap.INCHES_PER_TILE, ObstacleMap.INCHES_PER_TILE/2.)
                ),
                new RectangleObstacle(
                        3, 1. * ObstacleMap.INCHES_PER_TILE,
                        new Translation2d(-1*ObstacleMap.INCHES_PER_TILE, ObstacleMap.INCHES_PER_TILE/2.)
                ),
                new RectangleObstacle(
                        3, 1. * ObstacleMap.INCHES_PER_TILE,
                        new Translation2d(ObstacleMap.INCHES_PER_TILE, ObstacleMap.INCHES_PER_TILE/2.)
                ),
                new RectangleObstacle(
                        3, 1. * ObstacleMap.INCHES_PER_TILE,
                        new Translation2d(2*ObstacleMap.INCHES_PER_TILE, ObstacleMap.INCHES_PER_TILE/2.)
                ),
                // middle bar to add / remove
                new RectangleObstacle(
                        ObstacleMap.INCHES_PER_TILE * 2, 3,
                        new Translation2d(0, ObstacleMap.INCHES_PER_TILE/2.)
                ),
        };
    }
}
