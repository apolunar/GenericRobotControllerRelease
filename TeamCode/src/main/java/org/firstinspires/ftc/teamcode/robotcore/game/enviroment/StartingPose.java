package org.firstinspires.ftc.teamcode.robotcore.game.enviroment;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.math.util.Units;

import lombok.Getter;

public enum StartingPose {
    BLUE_AUDIENCE(new Pose2d(
            Units.feetToInches(-4.875),
            Units.feetToInches(-3.3),
            Rotation2d.fromDegrees(0))
    ),
    BLUE_BACKSTAGE(new Pose2d(
            Units.feetToInches(-4.875),
            Units.feetToInches(3.3),
            Rotation2d.fromDegrees(0))
    ),
    RED_AUDIENCE(new Pose2d(
            Units.feetToInches(5.16),
            Units.feetToInches(-3.15),
            Rotation2d.fromDegrees(180))
    ),
    RED_BACKSTAGE(new Pose2d(
            Units.feetToInches(5.16),
            Units.feetToInches(3.15),
            Rotation2d.fromDegrees(180))
    );

    public final Pose2d pose;
    StartingPose(Pose2d pose) {
        this.pose = pose;
    }
}
