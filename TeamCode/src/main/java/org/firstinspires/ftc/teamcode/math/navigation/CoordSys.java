package org.firstinspires.ftc.teamcode.math.navigation;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

/*
          ---FIELD---
               +Y
  ┌─────────────────────────┐
  │            ▲            │
  │            │            │
  │            │            │
  │            │            │
  │            │            │
  │            │            │
  │            │            │
-X│            └───────────►│+X
  │                         │
  │                         │
  │                         │
  │                         │
  │                         │
  │                         │
  └─────────────────────────┘
               -Y
*/
public final class CoordSys {
    private CoordSys() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static Pose2d robotToField(Pose2d robotCoords) {
        return new Pose2d();
    }

    public static Pose2d fieldToRobot(Pose2d fieldCoords) {
        return new Pose2d(fieldCoords.getY(), fieldCoords.getX(), fieldCoords.getRotation().rotateBy(Rotation2d.fromDegrees(90)));
    }

    public static Pose2d fieldToRobot(Translation2d fieldCoords) {
        return new Pose2d(fieldCoords.getY(), fieldCoords.getX(), Rotation2d.fromDegrees(90));
    }
}
