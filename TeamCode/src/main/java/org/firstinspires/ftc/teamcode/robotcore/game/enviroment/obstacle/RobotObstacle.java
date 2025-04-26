package org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle;

import com.arcrobotics.ftclib.geometry.Translation2d;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Getter
@RequiredArgsConstructor
public class RobotObstacle implements Obstacle {
    private double maxSize = 18; // DO NOT HIT IT!
    @Setter private Translation2d center;

    @Override
    public void scale(double scalar) {
        maxSize *= scalar;
    }

    @Override
    public void expand(double addend) {
        maxSize += addend;
    }

    @Override
    public double getMinSize() {
        return maxSize;
    }
}
