package org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import lombok.Getter;
import lombok.Setter;

@Getter
public class RectangleObstacle implements Obstacle {
    @Setter private double sizeX, sizeY, height;
    @Setter private Rotation2d rotation;
    @Setter private Translation2d center;

    public RectangleObstacle(double sizeX, double sizeY, double height, Rotation2d rotation, Translation2d center) {
        this.sizeX    = sizeX;
        this.sizeY    = sizeY;
        this.height   = height;
        this.rotation = rotation;
        this.center   = center;
    }

    public RectangleObstacle(double sizeX, double sizeY, Rotation2d rotation, Translation2d center) {
        this(sizeX, sizeY, 0, rotation, center);
    }

    public RectangleObstacle(double sizeX, double sizeY, Translation2d center) {
        this(sizeX, sizeY, 0, Rotation2d.fromDegrees(0), center);
    }

    @Override
    public void scale(double scalar) {
        sizeX *= scalar;
        sizeY *= scalar;
    }

    @Override
    public void expand(double addend) {
        sizeX += addend;
        sizeY += addend;
    }

    @Override
    public double getMaxSize() {
        return Math.max(sizeX, sizeY);
    }
    @Override
    public double getMinSize() {
        return Math.min(sizeX, sizeY);
    }
}
