package org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle;
import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@RequiredArgsConstructor
public class CircleObstacle implements Obstacle {
    private double radius;
    @Getter @Setter private Translation2d center;

    @Override
    public void scale(double scalar) {
        radius *= scalar;
    }

    @Override
    public void expand(double addend) {
        radius += addend;
    }

    @Override
    public double getMaxSize() {
        return radius;
    }

    @Override
    public double getMinSize() {
        return radius;
    }

}
