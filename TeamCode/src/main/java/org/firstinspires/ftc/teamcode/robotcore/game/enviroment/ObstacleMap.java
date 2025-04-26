package org.firstinspires.ftc.teamcode.robotcore.game.enviroment;

import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle.Obstacle;

public interface ObstacleMap {
    double INCHES_PER_TILE = 23.75;

    Obstacle[] getObstacles();
}
