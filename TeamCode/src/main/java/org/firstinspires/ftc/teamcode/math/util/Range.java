package org.firstinspires.ftc.teamcode.math.util;

/**
 * Range functions for util
 */
public class Range {
    public static boolean valueWithinPercentage(double value, double condition, double range) {
        return Math.abs(value) >= (Math.abs(condition) - Math.abs(condition)*range) && Math.abs(value) <= (Math.abs(condition) + Math.abs(condition)*range);
    }

    public static double lerp(double a, double b, double f)
    {
        return a * (1.0 - f) + (b * f);
    }
}
