package org.firstinspires.ftc.teamcode.demos;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle.Obstacle;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.obstacle.RectangleObstacle;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

import java.util.ArrayList;
import java.util.Comparator;

import lombok.Getter;
import lombok.Setter;

@Config
public class ScannerSubsystem extends SubsystemBase implements RobotSubsystem {
    // CONFIG
    private static final String SCANNER_SERVO_NAME = "scannerServo";
    private static final String SCANNER_ENCODER_NAME = "scannerEnc";
    private static final String SCANNER_DISTANCE_NAME = "scannerDist";

    public static final DistanceUnit DIST_UNIT = DistanceUnit.INCH;

    public static final double SCAN_POWER = 1;
    public static final double RAD_PER_TICK = 1; // TODO
    public static final double GAP_ANGLE_DEG     = 5;
    public static final double CALIBRATE_POWER   = 0.2;
    public static final double CALIBRATE_MIN_DIST = 1; // INCHES

    private double encoderDistOffset = GAP_ANGLE_DEG / 2;

    // HARDWARE
    private final CRServo scanner;
    private final Motor.Encoder encoder;
    private final Rev2mDistanceSensor distance;

    private final Obstacle[] obstacles;

    @Setter @Getter
    private boolean scanning = false;
    private boolean calibrated = false;

    private ArrayList<Pair<Double, Double>> scanHistory = new ArrayList<>();

    public ScannerSubsystem(CRServo scanner, Motor.Encoder encoder, Rev2mDistanceSensor distance, ObstacleMap obstacleMap) {
        this.scanner  = scanner;
        this.encoder  = encoder;
        this.distance = distance;

        encoder.reset();

        this.obstacles = obstacleMap == null ? new Obstacle[0] : obstacleMap.getObstacles();
    }

    public ScannerSubsystem(HardwareMap hMap) {
        this(hMap.get(CRServo.class, SCANNER_SERVO_NAME), new Motor(hMap, SCANNER_ENCODER_NAME).encoder, hMap.get(Rev2mDistanceSensor.class, SCANNER_DISTANCE_NAME), null);
    }

    @Override
    public void periodic() {
        if (!calibrated) {
            calibrateEncoder();
            return;
        }

        if (!scanning) return;

        double scanAngle = encoder.getDistance() + encoderDistOffset;

        if (scanAngle >= 2*Math.PI) {
            scanAngle = 0;
            encoder.reset();
            scanHistory.clear();
        }

        scanner.setPower(SCAN_POWER);

        double scanDistance = distance.getDistance(DIST_UNIT);

        scanHistory.add(new Pair<>(scanAngle, scanDistance));
    }

    private final ElapsedTime gapTimer = new ElapsedTime();
    private final ArrayList<Pair<Double, Double>> gapPositions = new ArrayList<>();
    private boolean firstPass = true;
    private int numPasses;

    /**
     * It is assumed that:
     *  - There is a hole of ~5deg size at the 180deg mark in the back of the scanner
     *  - Scanner walls are less than 1 inch away
     * TODO:
     *  - Figure out where "zero" / 180deg is so angle is scanAngle is consistent
     *
     *          x      xx      x
     *         xxxxx5degxxxxxxx
     *          x           xx
     *                ▲
     *         1gap   │    2gap
     *      xxxxx     │       xxxx
     *     xx         │           xxx
     *    xx     ┌────┴─────┐       xx
     *   xx      │  xxxxxx  │         x
     * 4gap      │  xxxxxx  │        3gap
     *           │  xxxxxx  │
     *           │  xxxxxx  │
     *           └──────────┘
     */
    private void calibrateEncoder() {
        if (numPasses < 6) {
            scanner.setPower(CALIBRATE_POWER);

            double scanTicks    = encoder.getPosition();
            double scanDistance = distance.getDistance(DIST_UNIT);

            if (scanDistance > CALIBRATE_MIN_DIST) {
                if (firstPass) { // START GAP
                    gapPositions.add(new Pair<>(gapTimer.seconds(), scanTicks));
                    gapTimer.reset();

                    firstPass = false;
                    numPasses++;
                }
            } else {
                if (!firstPass) { // END GAP
                    gapPositions.add(new Pair<>(gapTimer.seconds(), scanTicks));
                    gapTimer.reset();

                    firstPass = true;
                    numPasses++;
                }
            }
        } else {
            scanner.setPower(0);

            gapPositions.remove(0); // Remove gap of unknown starting position

            Pair<Double, Double> minDiffPair = gapPositions.stream().min(
                    Comparator.comparingDouble(timeTicksPair -> timeTicksPair.first)
            ).orElse(null); // Find pair with smallest delta time as small gap

            if (minDiffPair == null) {
                RobotLog.dd(this.m_name, "Could not calibrate!");
                // Try again
                gapPositions.clear();
                numPasses = 0;
                firstPass = true;
                gapTimer.reset();
                return;
            }

            double gapStartTicks = gapPositions.get(gapPositions.indexOf(minDiffPair) - 1).second;
            double gapEndTicks   = minDiffPair.second;

            encoder.setDistancePerPulse(
                    Math.toRadians(GAP_ANGLE_DEG) / (gapStartTicks - gapEndTicks)
            );
            encoder.reset();

            calibrated = true; // Start measuring data
        }
    }

    /**
     * TODO: Use scanHistory to look for things that are not part of the obstacle map given:
     *  - scanner offset
     *  - scanner angle
     *  - scanner distance
     *  - robot Pose2d
     */
    public RectangleObstacle[] getInferredObstacles(Pose2d robotPose) {

        return null;
    }
}
