package org.firstinspires.ftc.teamcode.math.util;

/**
 * Color functions for util
 */
public class Color {
//    public static float[] RGBToHSV()
    public static double[] convertRGBtoHSV(int red, int green, int blue) {
        double[] hsv = new double[3];

        // Normalize RGB values to be in the range 0.0 - 1.0
        double normRed = red / 255.0;
        double normGreen = green / 255.0;
        double normBlue = blue / 255.0;

        // Find the maximum and minimum values among RGB components
        double max = Math.max(normRed, Math.max(normGreen, normBlue));
        double min = Math.min(normRed, Math.min(normGreen, normBlue));

        // Calculate the hue
        if (max == min) {
            hsv[0] = 0; // No color change
        } else if (max == normRed) {
            hsv[0] = (60 * ((normGreen - normBlue) / (max - min)) + 360) % 360;
        } else if (max == normGreen) {
            hsv[0] = 60 * ((normBlue - normRed) / (max - min)) + 120;
        } else if (max == normBlue) {
            hsv[0] = 60 * ((normRed - normGreen) / (max - min)) + 240;
        }

        // Calculate the saturation
        if (max == 0) {
            hsv[1] = 0;
        } else {
            hsv[1] = 1 - (min / max);
        }

        // Calculate the value
        hsv[2] = max;

        return hsv;
    }
}
