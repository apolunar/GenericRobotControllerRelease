package org.firstinspires.ftc.teamcode.demos;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.zyxOrientation;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

import lombok.Getter;

/**
 * Inertial Measurement Unit
 *     - inertial direction
 *     - roll/pitch/yaw info
 */
@Config
public class IMUSubsystem extends SubsystemBase implements RobotSubsystem {

    // CONFIG
    public static String IMU_NAME = "imu";

    private static final double X_ROTATION = 0;
    private static final double Y_ROTATION = 0;
    private static final double Z_ROTATION = 0;
    private static final AxesReference AXES_REFERENCE = AxesReference.INTRINSIC;
    private static final AxesOrder AXES_ORDER = AxesOrder.ZYX;

    // HARDWARE
    @Getter private final IMU imu;

    public IMUSubsystem(IMU imu) {
        Orientation hubOrientation = zyxOrientation(Z_ROTATION, Y_ROTATION, X_ROTATION);
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(hubOrientation);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        this.imu = imu;
    }

    public IMUSubsystem(HardwareMap hMap) {
        this(hMap.get(IMU.class, IMU_NAME));
    }

    public double getYaw(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public double getYaw() {
        return getYaw(AngleUnit.RADIANS);
    }

    public double getYawVelocity(AngleUnit angleUnit) {
        return imu.getRobotAngularVelocity(angleUnit).zRotationRate;
    }

    public double getYawVelocity() {
        return getYawVelocity(AngleUnit.RADIANS);
    }
}
