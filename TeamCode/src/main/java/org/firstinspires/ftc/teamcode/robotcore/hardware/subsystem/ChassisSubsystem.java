package org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.OdoWheelSpeeds;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.util.Units;
import org.firstinspires.ftc.teamcode.robotcore.drivebase.MyMecanumDrive;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.StartingPose;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

import lombok.Getter;

@Config
public class ChassisSubsystem extends SubsystemBase implements RobotSubsystem {
    // CONFIG
    private static final String LEFT_FRONT_MOTOR_NAME = "leftFront";  // LEFT_ENC
    private static final String RIGHT_FRONT_MOTOR_NAME = "rightFront"; // RIGHT_ENC
    private static final String LEFT_BACK_MOTOR_NAME = "leftBack";
    private static final String RIGHT_BACK_MOTOR_NAME = "rightBack";  // HORIZONTAL_ENC
//    private static final String OCTOQUAD_NAME = "octoquad";
    private static final String PTO_NAME = "pto";
//    private static final int LEFT_FRONT_OCTOQUAD = 0;
//    private static final int LEFT_BACK_OCTOQUAD = 1;
//    private static final int RIGHT_BACK_OCTOQUAD = 2;
//    private static final int RIGHT_FRONT_OCTOQUAD = 3;

    private static final Motor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR =
            Motor.ZeroPowerBehavior.BRAKE;

    // Drive model parameters (inches)
    public static double TRACK_WIDTH = Units.millimetersToInches(251.5);
    public static double CENTER_WHEEL_OFFSET = Units.millimetersToInches(123.2);
    public static double O_WHEEL_RADIUS = Units.millimetersToInches(17.5);
    public static double M_WHEEL_RADIUS = Units.millimetersToInches(48);

    // Drive model parameters (inches)
    // Divided by clicks per rotation, however this value is tuned
    // Pushed the robot forward two tiles, averaged left and right tick values
    private static final double FORWARD_IN_PER_TICK = 50 / ((94400 + 93188) / 2.); // 48./((90882+90265)/2.);  // calculation: WHEEL_DIAMETER * Math.PI / 2786.0
    private static final double LATERAL_IN_PER_TICK = 50 / 93764.;
    private static final double RADIANS_PER_TICK = 2*Math.PI/((28_084+26_722)/2.);

    private static final double MECANUM_IN_PER_TICK = 0; // NOTE: only for use with OctoQuad encoders

    // Old feedforward scheme
//    public static double MOTOR_EFFICIENCY_X = 0.77;
//    public static double MOTOR_EFFICIENCY_Y = 0.34;
//
//    // RPM * WHEEL DIA (MM) * PI / MM_TO_IN / MIN_TO_SEC * EFF
//    public static double MAX_X_LINEAR_VELOCITY = 435 * 96 * Math.PI / (25.4 * 60) * MOTOR_EFFICIENCY_X; // TPS 435*96*Math.PI/60_000
//    public static double MAX_Y_LINEAR_VELOCITY = 435 * 96 * Math.PI / (25.4 * 60) * MOTOR_EFFICIENCY_Y; // TPS
    public static double MAX_RAD_PER_SECOND = 8.284;
    public static double MAX_ROTATIONAL_VELOCITY = 93560; // TPS (ticks per second)

    private static final double FEEDFORWARD_X_K_V = 7.92e-03;
    private static final double FEEDFORWARD_X_K_V_V = 8e-05;
    private static final double FEEDFORWARD_X_K_A = 0;
    private static final double FEEDFORWARD_X_K_S = 0.143;

    private static final double FEEDFORWARD_Y_K_V = 0.0101;
    private static final double FEEDFORWARD_Y_K_V_V = 1.53e-04;
    private static final double FEEDFORWARD_Y_K_A = 0;
    private static final double FEEDFORWARD_Y_K_S = 0.203;

    private static double BACK_MOTOR_CORRECTION = 1;
    private static double FRONT_MOTOR_CORRECTION = 1;

    // Locations of the wheels relative to the robot center
    public static Translation2d FRONT_LEFT_LOCATION =
            new Translation2d(Units.millimetersToInches(144), -Units.millimetersToInches(156));
    public static Translation2d FRONT_RIGHT_LOCATION =
            new Translation2d(Units.millimetersToInches(144), Units.millimetersToInches(156));
    public static Translation2d BACK_LEFT_LOCATION =
            new Translation2d(-Units.millimetersToInches(156), -Units.millimetersToInches(156));
    public static Translation2d BACK_RIGHT_LOCATION =
            new Translation2d(-Units.millimetersToInches(156), Units.millimetersToInches(156));

//    @Getter private final ArrayList<Pose2d> poseHistory = new ArrayList<>();

    // HARDWARE
    @Getter
    private final MotorEx.Encoder leftOdometer, rightOdometer, centerOdometer;
    @Getter
    private final MotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    @Getter
    private final Servo PTO;
//    @Getter
//    private final OctoQuad octoQuad;
    @Getter
    private final OdometrySubsystem odometrySubsystem;
    @Getter
    private final HolonomicOdometry holonomicOdometry;
    @Getter
    private final IMU imu;

    @Getter
    private final MyMecanumDrive drive;
    @Getter
    private final MecanumDriveKinematics driveKinematics;

    public static double vxkP = 7;
    public static double vxkI = 0.03;
    public static double vxkD = 0.04;

    public static double vykP = 7;
    public static double vykI = 0.03;
    public static double vykD = 0.05;

    public static double wkP = 0.5;
    public static double wkI = 5; // still lots of oscillations
    public static double wkD = 0.5;

    public static PIDController vxController = new PIDController(vxkP, vxkI, vxkD);
    public static PIDController vyController = new PIDController(vykP, vykI, vykD);
    public static PIDController omegaController = new PIDController(wkP, wkI, wkD);

    public ChassisSubsystem(MotorEx leftFrontMotor, MotorEx rightFrontMotor,
                            MotorEx leftBackMotor, MotorEx rightBackMotor, Servo PTO,
                            IMU imu, Pose2d startingPose) {
        leftFrontMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        rightFrontMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        leftBackMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);
        rightBackMotor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        leftFrontMotor.setInverted(true);
        rightFrontMotor.setInverted(true);
        leftBackMotor.setInverted(true);
        rightBackMotor.setInverted(true);

        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;

        this.PTO = PTO;

        // Creating my kinematics object using the wheel locations
        driveKinematics = new MecanumDriveKinematics(
                FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION,
                BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION
        );

        // Forward is where the front-facing camera is pointing
//        this.octoQuad = octoQuad;
        leftOdometer = leftBackMotor.encoder;
        rightOdometer = rightFrontMotor.encoder;
        centerOdometer = rightBackMotor.encoder;

        // Create our odometry object and subsystem
        holonomicOdometry = new HolonomicOdometry(
                () -> leftOdometer.getPosition() * FORWARD_IN_PER_TICK,
                () -> rightOdometer.getPosition() * FORWARD_IN_PER_TICK,
                () -> centerOdometer.getPosition() * LATERAL_IN_PER_TICK,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        if (startingPose != null) {
            RobotLog.dd(this.m_name, "Updating starting pose to : %s", startingPose);
            holonomicOdometry.updatePose(startingPose);
        }

//        leftOdometer.setDirection(Motor.Direction.REVERSE);
        rightOdometer.setDirection(Motor.Direction.REVERSE);
//        centerOdometer.setDirection(Motor.Direction.REVERSE);

        leftOdometer.reset();
        rightOdometer.reset();
        centerOdometer.reset();

        odometrySubsystem = new OdometrySubsystem(holonomicOdometry);

        this.imu = imu;

        drive = new MyMecanumDrive(leftFrontMotor, rightFrontMotor,
                leftBackMotor, rightBackMotor, BACK_MOTOR_CORRECTION, FRONT_MOTOR_CORRECTION);
    }

    /**
     * Creates a new ChassisSubsystem with the hardware map and configuration names, starting pose, and alliance.
     * The odometry will optionally be updated with the starting pose to maintain the coordinate system.
     */
    public ChassisSubsystem(HardwareMap hMap, Pose2d startingPose) {
        this(new MotorEx(hMap, LEFT_FRONT_MOTOR_NAME), new MotorEx(hMap, RIGHT_FRONT_MOTOR_NAME),
                new MotorEx(hMap, LEFT_BACK_MOTOR_NAME), new MotorEx(hMap, RIGHT_BACK_MOTOR_NAME),
                hMap.get(Servo.class, PTO_NAME),
                hMap.get(IMU.class, "imu"), startingPose);
    }

    public ChassisSubsystem(HardwareMap hMap, StartingPose startingPose) {
        this(hMap, startingPose.pose);
    }

    public ChassisSubsystem(HardwareMap hMap) {
        this(hMap, (Pose2d) null);
    }

    public OdoWheelSpeeds getOdoWheelSpeeds() {
        return new OdoWheelSpeeds(leftOdometer.getCorrectedVelocity(), rightOdometer.getCorrectedVelocity(), centerOdometer.getCorrectedVelocity());
    }

//    public MecanumDriveWheelSpeeds getMecanumWheelSpeedsTicks() {
//        short[] wheelVelocities = octoQuad.readAllVelocities();
//        return new MecanumDriveWheelSpeeds(wheelVelocities[0], wheelVelocities[3], wheelVelocities[1], wheelVelocities[2]); // array indices are hardware-defined
//    }

//    public MecanumDriveWheelSpeeds getMecanumWheelSpeeds() {
//        MecanumDriveWheelSpeeds wheelSpeeds = getMecanumWheelSpeedsTicks();
//        return new MecanumDriveWheelSpeeds(
//                wheelSpeeds.frontLeftMetersPerSecond * MECANUM_IN_PER_TICK,
//                wheelSpeeds.frontRightMetersPerSecond * MECANUM_IN_PER_TICK,
//                wheelSpeeds.rearLeftMetersPerSecond * MECANUM_IN_PER_TICK,
//                wheelSpeeds.rearRightMetersPerSecond * MECANUM_IN_PER_TICK
//        );
//    }

    /**
     * Updates the robot's pose from odometry.
     */
    @Override
    public void periodic() {
        holonomicOdometry.updatePose();
        // Pose history?
//        poseHistory.add(holonomicOdometry.getPose());
//        RobotLog.dd(this.m_name, "Latest pose : %s", holonomicOdometry.getPose());
    }

    public Pose2d getLatestPose() {
        return holonomicOdometry.getPose();
    }

    /**
     * Drives the robot using 2D controls relative to the odo heading.
     *
     * @param lateralSpeed  the commanded lateral movement
     * @param forwardSpeed  the commanded forward movement
     * @param rotationSpeed the commanded rotation speed
     * @param heading       the current heading of the robot in degrees
     */
    public void driveFieldCentric(double lateralSpeed, double forwardSpeed,
                                  double rotationSpeed, double heading) {
        double omega = rotationSpeed;
//        double omega;
//
//        LogWrapper.telemetry.addData("rotationSpeed", rotationSpeed);
//
//        if (lateralSpeed != 0 || forwardSpeed != 0 || rotationSpeed != 0) {
//            double measuredRotationVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate;
//            double targetRotationVelocity = rotationSpeed * MAX_RAD_PER_SECOND;
//
//            RobotLog.dd(this.m_name, "measured : %s", measuredRotationVelocity);
//            RobotLog.dd(this.m_name, "target   : %s", targetRotationVelocity);
//
//            /*
//            // Odo version
//            double measuredRotationVelocity = odoWheelSpeeds.leftMetersPerSecond + odoWheelSpeeds.rightMetersPerSecond;
//
//            RobotLog.dd(this.m_name, "left     : %s", odoWheelSpeeds.leftMetersPerSecond);
//            RobotLog.dd(this.m_name, "right    : %s", odoWheelSpeeds.rightMetersPerSecond);
//            */
//
//            // For rapid iteration with dashboard
//            omegaController = new PIDController(wkP, wkI, wkD);
//
//            double omagaFeedback = omegaController.calculate(
//                    measuredRotationVelocity,
//                    targetRotationVelocity
//            ) / MAX_RAD_PER_SECOND; // normalize feedback
//
//            omega = rotationSpeed - omagaFeedback;
//
//            RobotLog.dd(this.m_name, "w feedback  : %s", omagaFeedback);
//            RobotLog.dd(this.m_name, "omega rad/s : %s", omega);
//        } else {
//            omega = 0;
//        }

        drive.driveFieldCentric(lateralSpeed, forwardSpeed,
                omega, heading);
    }


    public void driveChassisSpeeds(ChassisSpeeds targetChassisSpeeds) {
        // TODO: Fix velocity PID
//        OdoWheelSpeeds odoWheelSpeeds = getOdoVelocities();
//        double measuredRotationVelocity = (odoWheelSpeeds.leftMetersPerSecond + odoWheelSpeeds.rightMetersPerSecond) / TRACK_WIDTH;
//
//        ChassisSpeeds measuredChassisSpeeds = new ChassisSpeeds(
//                odoWheelSpeeds.leftMetersPerSecond,
//                odoWheelSpeeds.rightMetersPerSecond,
//                measuredRotationVelocity
//        );
//
//        double vxFeedback = vxController.calculate(
//                measuredChassisSpeeds.vxMetersPerSecond,
//                targetChassisSpeeds.vxMetersPerSecond
//        );
//        double vyFeedback = vyController.calculate(
//                measuredChassisSpeeds.vyMetersPerSecond,
//                targetChassisSpeeds.vyMetersPerSecond
//        );
//        double vtFeedback = vtController.calculate(
//                measuredChassisSpeeds.omegaRadiansPerSecond,
//                targetChassisSpeeds.omegaRadiansPerSecond
//        );
//
//        double strafeSpeed  = (targetChassisSpeeds.vyMetersPerSecond + vxFeedback) / MAX_Y_LINEAR_VELOCITY;
//        double forwardSpeed = (targetChassisSpeeds.vxMetersPerSecond + vyFeedback) / MAX_X_LINEAR_VELOCITY;
//        double turnSpeed    = (targetChassisSpeeds.omegaRadiansPerSecond + vtFeedback) / MAX_RAD_PER_SECOND;
//
//        RobotLog.vv(this.getClass().getSimpleName(), "Calculate feedbacks  : (vt feedback: %s vx feedback: %s vy feedback: %s)", vtFeedback, vxFeedback, vyFeedback);
//        RobotLog.vv(this.getClass().getSimpleName(), "Calculate outputs    : (strafe speed: %s forward speed: %s turn speed: %s)", strafeSpeed, forwardSpeed, turnSpeed);

//        drive.driveRobotCentric(
//                strafeSpeed,
//                forwardSpeed,
//                turnSpeed
//        );
//        drive.driveRobotCentric(
//                targetChassisSpeeds.vyMetersPerSecond / MAX_Y_LINEAR_VELOCITY,
//                targetChassisSpeeds.vxMetersPerSecond / MAX_X_LINEAR_VELOCITY,
//                targetChassisSpeeds.omegaRadiansPerSecond / MAX_RAD_PER_SECOND
//        );
        // TODO
        driveChassisSpeeds(targetChassisSpeeds, 0);
    }

    public void driveChassisSpeeds(ChassisSpeeds targetChassisSpeeds, double acceleration) {
        // The control system is non-linear
        drive.driveRobotCentric(
                 -(targetChassisSpeeds.vyMetersPerSecond * (FEEDFORWARD_Y_K_V_V * targetChassisSpeeds.vyMetersPerSecond * Math.signum(targetChassisSpeeds.vyMetersPerSecond)
                            + FEEDFORWARD_Y_K_V)
                        + FEEDFORWARD_Y_K_A * acceleration
                        + FEEDFORWARD_Y_K_S * Math.signum(targetChassisSpeeds.vyMetersPerSecond)),
                 -(targetChassisSpeeds.vxMetersPerSecond * (FEEDFORWARD_X_K_V_V * targetChassisSpeeds.vxMetersPerSecond * Math.signum(targetChassisSpeeds.vxMetersPerSecond)
                            + FEEDFORWARD_X_K_V)
                        + FEEDFORWARD_X_K_A * acceleration
                        + FEEDFORWARD_X_K_S * Math.signum(targetChassisSpeeds.vxMetersPerSecond)),
                -targetChassisSpeeds.omegaRadiansPerSecond / MAX_RAD_PER_SECOND // TODO: Feedforward rotation...?
        );
    }

    /**
     * Drives the robot using 2D controls.
     *
     * @param lateralSpeed  the commanded lateral movement
     * @param forwardSpeed  the commanded forward movement
     * @param rotationSpeed the commanded turn speed
     */
    public void drive(double lateralSpeed, double forwardSpeed, double rotationSpeed) {
        driveFieldCentric(lateralSpeed, forwardSpeed, rotationSpeed, 0);
    }

    public void driveWheelSpeeds(MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds) {
        drive.driveWithMotorPowers(
                mecanumDriveWheelSpeeds.frontLeftMetersPerSecond / M_WHEEL_RADIUS,
                mecanumDriveWheelSpeeds.frontRightMetersPerSecond / M_WHEEL_RADIUS,
                mecanumDriveWheelSpeeds.rearLeftMetersPerSecond / M_WHEEL_RADIUS,
                mecanumDriveWheelSpeeds.rearRightMetersPerSecond / M_WHEEL_RADIUS
        );
    }

    public void drivePTO(double position) {
        PTO.setPosition(position);
    }

    public Motor[] getMotors() {
        return new Motor[]{leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor};
    }
}
