package org.firstinspires.ftc.teamcode.demos;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.logging.LogWrapper;
import org.firstinspires.ftc.teamcode.robotcore.game.element.Pixel;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

/**
 * Moves the pixel to the delivery (attached to the slide).
 */
@Config
public class SpatulaSubsystem extends SubsystemBase implements RobotSubsystem {
    // CONFIG
    private static final double LOW_SERVO = 0.15;

    private static final double MIN_SERVO_L = 0.500 - 0.29;
    private static final double MAX_SERVO_L = 0.500 + 0.36;
    private static final double MID_SERVO_L = (MIN_SERVO_L + MAX_SERVO_L) / 2.1;
    private static final double LOW_SERVO_L = MIN_SERVO_L - (MIN_SERVO_L - MAX_SERVO_L)*LOW_SERVO;

    private static final double MIN_SERVO_R = 0.500 + 0.29;
    private static final double MAX_SERVO_R = 0.500 - 0.36;
    private static final double MID_SERVO_R = (MIN_SERVO_R + MAX_SERVO_R) / 2.1;
    private static final double LOW_SERVO_R = MIN_SERVO_R - (MIN_SERVO_R - MAX_SERVO_R)*LOW_SERVO;
    public enum Position {
        MIN(MIN_SERVO_L, MIN_SERVO_R, 0),
        MAX(MAX_SERVO_L, MAX_SERVO_R, 1),
        MID(
                MID_SERVO_L,
                MID_SERVO_R,
                0.5
        ),
        LOW(LOW_SERVO_L, LOW_SERVO_R, LOW_SERVO);

        public final double spatLPosition;
        public final double spatRPosition;
        public final double spatPosition;
        Position(double spatLPosition, double spatRPosition, double spatPosition) {
            this.spatLPosition = spatLPosition;
            this.spatRPosition = spatRPosition;
            this.spatPosition = spatPosition;
        }
    }
    public static String BACK_COLOR_SENSOR_NAME  = "colorBack";
    public static String FRONT_COLOR_SENSOR_NAME = "colorFront";
    public static String SPATULA_L_SERVO_NAME    = "spatL";
    public static String SPATULA_R_SERVO_NAME    = "spatR";

    // HARDWARE
    private final Servo spatulaLeft;
    private final Servo spatulaRight;
    private final SensorColor sensorColorBack;
    private final SensorColor sensorColorFront;

    public SpatulaSubsystem(Servo spatulaLeft, Servo spatulaRight, SensorColor sensorColorBack, SensorColor sensorColorFront) {
        this.spatulaLeft      = spatulaLeft;
        this.spatulaRight     = spatulaRight;
        this.sensorColorBack  = sensorColorBack;
        this.sensorColorFront = sensorColorFront;
    }

    /**
     * Creates a new SpatulaSubsystem with the hardware map and configuration names.
     */
    public SpatulaSubsystem(HardwareMap hMap) {
        this(hMap.get(Servo.class, SPATULA_L_SERVO_NAME), hMap.get(Servo.class, SPATULA_R_SERVO_NAME),
                new SensorColor(hMap, BACK_COLOR_SENSOR_NAME), new SensorColor(hMap, FRONT_COLOR_SENSOR_NAME));
    }

    public Pixel.PixelColor[] getPixels() {
        // TODO: Why do I have to multiply last two by 100?
        float[] backColorHsv = new float[3];
        Color.RGBToHSV(sensorColorBack.red(), sensorColorBack.green(), sensorColorBack.blue(), backColorHsv);
        backColorHsv[1] *= 100;
        LogWrapper.log(this.m_name, "Back color : (red: %s, green: %s, blue: %s)", sensorColorBack.red(), sensorColorBack.green(), sensorColorBack.blue());
        LogWrapper.log(this.m_name, "Back hsv : %s %s %s", backColorHsv[0], backColorHsv[1], backColorHsv[2]);

        Pixel.PixelColor backPixelColor = Pixel.PixelColor.closestPixel(backColorHsv, true);

        float[] frontColorHsv = new float[3];
        Color.RGBToHSV(sensorColorFront.red(), sensorColorFront.green(), sensorColorFront.blue(), frontColorHsv);
        frontColorHsv[1] *= 100;
        LogWrapper.log(this.m_name, "Front hsv : %s %s %s", frontColorHsv[0], frontColorHsv[1], frontColorHsv[2]);

        Pixel.PixelColor fronPixelColor = Pixel.PixelColor.closestPixel(frontColorHsv, false);

        return new Pixel.PixelColor[] {
                backPixelColor,
                fronPixelColor
        };
    }

    public void goToPosition(Position spatPosition) {
        spatulaLeft.setPosition(spatPosition.spatLPosition);
        spatulaRight.setPosition(spatPosition.spatRPosition);
    }

    public void interpolatePosition(double position) {
        double spatLPosition = MIN_SERVO_L + position*(MAX_SERVO_L-MIN_SERVO_L);
        double spatRPosition = MIN_SERVO_R + position*(MAX_SERVO_R-MIN_SERVO_R);

        LogWrapper.log(this.m_name, "Spatula going to position : (left: %s, right: %s)", spatLPosition, spatRPosition);

        spatulaLeft.setPosition(spatLPosition);
        spatulaRight.setPosition(spatRPosition);
    }
}
