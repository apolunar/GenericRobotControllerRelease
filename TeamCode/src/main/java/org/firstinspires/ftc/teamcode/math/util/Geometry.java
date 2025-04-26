package org.firstinspires.ftc.teamcode.math.util;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Point3;

import lombok.Getter;

/**
 * Random math functions that aren't in other libraries
 */
public class Geometry {
    public static class ThreeDimensional {
        public static double distanceBetween(float[] p1, float[] p2) {
            return Math.sqrt( Math.pow(p1[0] - p2[0], 2) + Math.pow(p1[1] - p2[1], 2) + Math.pow(p1[2] - p2[2], 2));
        }
    }

    public static class TwoDimensional {
        public static boolean poseWithinPercentage(Pose2d curPose, Pose2d targetPose, double percentage) {
            return (curPose.getX() / targetPose.getX()) <= percentage &&
                    (curPose.getY() / targetPose.getY()) <= percentage;
        }
    }

    public static class Polar {
        double angle;
        @Getter
        double magnitude;

        public Polar(double x, double y) {
            angle     = Math.atan2(y, x);
            magnitude = Math.hypot(x, y);
        }

        public double getAngle(@NonNull AngleUnit angleUnit) {
            return angleUnit.fromRadians(angle);
        }
    }
}
