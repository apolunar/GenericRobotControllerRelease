package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

import lombok.Getter;

/**
 * Drone launching system
 */
@Config
public class DroneSubsystem extends SubsystemBase implements RobotSubsystem {

    // CONFIG
    public static String DRONE_LAUNCH_SERVO_NAME = "droneServo";

    private static final double LAUNCH_POSITION = 0.4;

    // HARDWARE
    private final Servo droneLaunchServo;

    @Getter private boolean droneLaunched = false;

    /**
     * Creates a new DroneSubsystem.
     */
    public DroneSubsystem(Servo droneLaunchServo) {
        this.droneLaunchServo = droneLaunchServo;
    }

    /**
     * Creates a new DroneSubsystem with the hardware map and configuration names.
     */
    public DroneSubsystem(HardwareMap hMap) {
        this(hMap.get(Servo.class, DRONE_LAUNCH_SERVO_NAME));
    }

    /**
     * Launches the drone.
     */
    public void launch() {
        if (droneLaunchServo == null) return;

        droneLaunchServo.setPosition(LAUNCH_POSITION);

        droneLaunched = true;
    }
}
