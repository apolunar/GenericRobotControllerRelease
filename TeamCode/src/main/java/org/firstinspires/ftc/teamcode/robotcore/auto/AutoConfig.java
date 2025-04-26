package org.firstinspires.ftc.teamcode.robotcore.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.math.navigation.Navigator;
import org.firstinspires.ftc.teamcode.math.navigation.navigators.AStarNavigator;
import org.firstinspires.ftc.teamcode.math.util.Units;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.maps.CenterStageObstacleMap;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
@Config
public class AutoConfig {
    /**
     * Error
     */
    public static double SUBSYSTEM_ERROR = .075;
    public static double LIFT_SUBSYSTEM_ERROR = .01;

    /**
     * Default PID values
     */
    public static double xkP = 7;
    public static double xkI = 0.11;
    public static double xkD = 0.06;
    public static double ykP = 7;
    public static double ykI = 0.11;
    public static double ykD = 0.06;
    public static double tkP = 3;
    public static double tkI = 1.0;
    public static double tkD = 0.001;
    public static int ANGULAR_VEL = 600;
    public static int ANGULAR_ACCEL = 600;

    /**
     * Set pose tolerance, constraints, and PID values here
     */
    @Builder.Default
    public static HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(xkP, xkI, xkD), // vx controller
            new PIDController(ykP, ykI, ykD), // vy controller
            new ProfiledPIDController(tkP, tkI, tkD, new TrapezoidProfile.Constraints(ANGULAR_VEL, ANGULAR_ACCEL)) // omega controller
    );

    /**
     * Trajectory config
     */
    public static int VEL = 51;
    public static int ACCEL = 75;
    public static TrajectoryConfig TRAJECTORY_CONFIG = new TrajectoryConfig(VEL, ACCEL);
    public static Pose2d DEFAULT_POSE_TOLERANCE = new Pose2d(4.5, 4.5, Rotation2d.fromDegrees(3));
    public static double DEFAULT_TRAJECTORY_RUNTIME_TOLERANCE = 0.1; // in addition to initial estimate
    public static Navigator.RobotSize ROBOT_SIZE = new Navigator.RobotSize(18, 12);
    public static AStarNavigator FIELD_NAVIGATOR = new AStarNavigator(
            new CenterStageObstacleMap(),
            Units.feetToInches(12),
            ROBOT_SIZE,
            TRAJECTORY_CONFIG
    );

    @Builder.Default
    private int MAX_DEPTH = 2;
}
