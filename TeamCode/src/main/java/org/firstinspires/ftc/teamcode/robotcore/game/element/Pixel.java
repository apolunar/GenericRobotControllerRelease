package org.firstinspires.ftc.teamcode.robotcore.game.element;

import org.firstinspires.ftc.teamcode.math.util.Geometry;

public class Pixel {
    public PixelColor getPixelColor() {
        return pixelColor;
    }

    /**
     * All values are HSV
     */
    public enum PixelColor {
        WHITE(new float[]{151, 36, 7}, new float[]{188, 54, 5}),
        GREEN(new float[]{122, 68, 2}, new float[]{141, 68, 1}),
        PURPLE(new float[]{216, 43, 4}, new float[]{212, 63, 2}),
        YELLOW(new float[]{77, 77, 3}, new float[]{110, 52, 1}),
        NONE(new float[]{132, 35, 0}, new float[]{175, 52, 0}); // blue of the intake

        final float[] colorBack;
        final float[] colorFront;
        PixelColor(float[] colorBack, float[] colorFront) {
            this.colorBack  = colorBack;
            this.colorFront = colorFront;
        }

        public static PixelColor closestPixel(float[] hsv, boolean isSensorBack) {
            int minDist = Integer.MAX_VALUE;
            PixelColor minDistPixelColor = PixelColor.NONE;
            for (PixelColor pixelColor : PixelColor.values()) {
                double distBetween = Geometry.ThreeDimensional.distanceBetween(isSensorBack ? pixelColor.colorBack : pixelColor.colorFront, hsv);
                if (distBetween < minDist) {
                    minDist = (int) distBetween;
                    minDistPixelColor = pixelColor;
                }
            }
            return minDistPixelColor;
        }
    }

    private final PixelColor pixelColor;
    public Pixel(PixelColor pixelColor) {
        this.pixelColor = pixelColor;
    }
}
