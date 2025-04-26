package org.firstinspires.ftc.teamcode.math.navigation;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle.Obstacle;

import java.lang.reflect.Field;
import java.util.ArrayList;

import lombok.RequiredArgsConstructor;

public interface Navigator {
    @RequiredArgsConstructor
    class RobotSize {
        public final double sqSize;
        public final double height;
    }

    static void logObstacleMap(ArrayList<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles) {
            RobotLog.dd("NAVIGATOR", "Obstacle : %s", obstacle.getClass().getSimpleName());
            for (Field field : obstacle.getClass().getFields()) {
                try {
                    RobotLog.dd("NAVIGATOR", "  Name : %s Value : %s", field.getName(), field.get(obstacle));
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    ArrayList<Obstacle> generateObstacles(ObstacleMap obstacleMap);

    Trajectory search(Translation2d source, Translation2d target);

    void addObstacle(Obstacle obstacle);

    void removeObstacle(Obstacle obstacle);

    ArrayList<Obstacle> getObstacles();

    static double angleBetween(Translation2d origin, Translation2d afterOrigin) {
        double sourceDeltaY = -(origin.getY() - afterOrigin.getY());
        double sourceDeltaX = -(origin.getX() - afterOrigin.getX());

        // TODO: Flip sign based on direction of infinity
        return Math.atan2(sourceDeltaY, sourceDeltaX);
    }
}
