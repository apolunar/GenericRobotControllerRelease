package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.logging.LogWrapper;
import org.firstinspires.ftc.teamcode.robotcore.game.element.Pixel;
import org.firstinspires.ftc.teamcode.robotcore.game.element.Prop;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

/**
 * Control exterior lights
 */
@Config
public class BlinkinSubsystem extends SubsystemBase implements RobotSubsystem {
    // CONFIG
    private static final String BLINKIN_NAME = "blinkin";

    // HARDWARE
    private final RevBlinkinLedDriver blinkin;

    public BlinkinSubsystem(RevBlinkinLedDriver blinkin) {
        this.blinkin = blinkin;
    }

    public BlinkinSubsystem(HardwareMap hMap) {
        this(hMap.get(RevBlinkinLedDriver.class, BLINKIN_NAME));
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (blinkin == null) return;

        blinkin.setPattern(pattern);
    }

    public void setPixelPattern(Pixel.PixelColor[] pixelColors, int pixelNum) {
        LogWrapper.log(this.m_name, "Pixel color : (back: %s front: %s)", pixelColors[0], pixelColors[1]);

        RevBlinkinLedDriver.BlinkinPattern targetPattern;
        switch (pixelColors[pixelNum]) {
            case PURPLE:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                break;
            case GREEN:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                break;
            case YELLOW:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;
            case WHITE:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                break;
            default:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
        }
        blinkin.setPattern(targetPattern);
    }

    public void setPropPattern(Prop.PropPosition propPosition) {
        if (propPosition == null) { // TODO: This is bad
            return;
        }

        RevBlinkinLedDriver.BlinkinPattern targetPattern;
        switch (propPosition) {
            case LEFT:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.LIME;
                break;
            case CENTER:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
                break;
            case RIGHT:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
                break;
            case NONE:
            default:
                targetPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                break;
        }
        blinkin.setPattern(targetPattern);
    }
}
