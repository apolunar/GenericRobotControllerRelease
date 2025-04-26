package org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle;

import com.arcrobotics.ftclib.geometry.Translation2d;

/**
 * An interface for creating environmental obstacles
 * Thanks <a href="https://github.com/Froze-N-Milk/mercurialftc/blob/unstable/src/main/java/org/mercurialftc/mercurialftc/silversurfer/geometry/obstaclemap/ObstacleMap.java">bestie</a>
 */
public interface Obstacle {
    Translation2d getCenter();
    void setCenter(Translation2d newCenter);
    void scale(double scalar);
    void expand(double addend);

    double getMaxSize();
    double getMinSize();
}
