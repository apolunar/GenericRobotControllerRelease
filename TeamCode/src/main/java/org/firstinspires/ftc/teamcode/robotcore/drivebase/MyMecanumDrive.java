package org.firstinspires.ftc.teamcode.robotcore.drivebase;

import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;

/**
 * Better FTCLib MecanumDrive class
 * Now with wheel correction!
 * See <a href="https://github.com/FTCLib/FTCLib/blob/1c8995d09413b406e0f4aff238ea4edc2bb860c4/core/src/main/java/com/arcrobotics/ftclib/drivebase/MecanumDrive.java#L17">the original.</a>
 */
public class MyMecanumDrive extends RobotDrive {
    private double rightSideMultiplier;

    private Motor[] motors;

    private final double frontLeftCorrection, frontRightCorrection, backLeftCorrection, backRightCorrection;

    /**
     * Sets up the constructor for the mecanum drive.
     *
     * @param autoInvert Whether or not to automatically invert the right motors
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    public MyMecanumDrive(boolean autoInvert, Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight,
                        double frontLeftCorrection, double frontRightCorrection, double backLeftCorrection, double backRightCorrection) {
        motors = new Motor[]{frontLeft, frontRight, backLeft, backRight};
        setRightSideInverted(autoInvert);

        this.frontLeftCorrection  = frontLeftCorrection;
        this.frontRightCorrection = frontRightCorrection;
        this.backLeftCorrection   = backLeftCorrection;
        this.backRightCorrection  = backRightCorrection;
    }

    /**
     * Sets up the constructor for the mecanum drive.
     * Automatically inverts right side by default
     *
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    public MyMecanumDrive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        this(true, frontLeft, frontRight, backLeft, backRight, 1, 1, 1, 1);
    }

    public MyMecanumDrive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, double frontWheelCorrection, double backWheelCorrection) {
        this(true, frontLeft, frontRight, backLeft, backRight, frontWheelCorrection, frontWheelCorrection, backWheelCorrection, backWheelCorrection);
    }


    /**
     * Checks if the right side motors are inverted.
     *
     * @return true if the multiplier for the right side is equal to -1.
     */
    public boolean isRightSideInverted() {
        return rightSideMultiplier == -1.0;
    }

    /**
     * Sets the right side inversion factor to the specified boolean.
     *
     * @param isInverted If true, sets the right side multiplier to -1 or 1 if false.
     */
    public void setRightSideInverted(boolean isInverted) {
        rightSideMultiplier = isInverted ? -1.0 : 1.0;
    }

    /**
     * Sets the range of the input, see RobotDrive for more info.
     *
     * @param min The minimum value of the range.
     * @param max The maximum value of the range.
     */
    public void setRange(double min, double max) {
        super.setRange(min, max);
    }

    /**
     * Sets the max speed of the drivebase, see RobotDrive for more info.
     *
     * @param value The maximum output speed.
     */
    public void setMaxSpeed(double value) {
        super.setMaxSpeed(value);
    }

    /**
     * Stop the motors.
     */
    @Override
    public void stop() {
        for (Motor x : motors) {
            x.stopMotor();
        }
    }


    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, 0.0);
    }


    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param squareInputs Square joystick inputs for finer control
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed, boolean squareInputs) {
        strafeSpeed = squareInputs ? clipRange(squareInput(strafeSpeed)) : clipRange(strafeSpeed);
        forwardSpeed = squareInputs ? clipRange(squareInput(forwardSpeed)) : clipRange(forwardSpeed);
        turnSpeed = squareInputs ? clipRange(squareInput(turnSpeed)) : clipRange(turnSpeed);

        driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro
     */
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed,
                                  double turnSpeed, double gyroAngle) {
        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turnSpeed = clipRange(turnSpeed);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
        input = input.rotateBy(-gyroAngle);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[MotorType.kFrontLeft.value] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[MotorType.kFrontRight.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[MotorType.kBackLeft.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[MotorType.kBackRight.value] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[MotorType.kFrontLeft.value] += turnSpeed;
        wheelSpeeds[MotorType.kFrontRight.value] -= turnSpeed;
        wheelSpeeds[MotorType.kBackLeft.value] += turnSpeed;
        wheelSpeeds[MotorType.kBackRight.value] -= turnSpeed;

        normalize(wheelSpeeds);

        driveWithMotorPowers(
                wheelSpeeds[MotorType.kFrontLeft.value],
                wheelSpeeds[MotorType.kFrontRight.value],
                wheelSpeeds[MotorType.kBackLeft.value],
                wheelSpeeds[MotorType.kBackRight.value]
        );
    }

    /**
     * Drives the robot from the perspective of the driver. No matter the orientation of the
     * robot, pushing forward on the drive stick will always drive the robot away
     * from the driver.
     *
     * @param xSpeed       the horizontal speed of the robot, derived from input
     * @param ySpeed       the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     * @param gyroAngle    the heading of the robot, derived from the gyro
     * @param squareInputs Square the value of the input to allow for finer control
     */
    public void driveFieldCentric(double xSpeed, double ySpeed, double turnSpeed, double gyroAngle, boolean squareInputs) {
        xSpeed = squareInputs ? clipRange(squareInput(xSpeed)) : clipRange(xSpeed);
        ySpeed = squareInputs ? clipRange(squareInput(ySpeed)) : clipRange(ySpeed);
        turnSpeed = squareInputs ? clipRange(squareInput(turnSpeed)) : clipRange(turnSpeed);

        driveFieldCentric(xSpeed, ySpeed, turnSpeed, gyroAngle);
    }

    /**
     * Drives the motors directly with the specified motor powers.
     *
     * @param frontLeftSpeed    the speed of the front left motor
     * @param frontRightSpeed   the speed of the front right motor
     * @param backLeftSpeed     the speed of the back left motor
     * @param backRightSpeed    the speed of the back right motor
     */
    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        // TODO: Maybe correction should be a property of the Motor?
        motors[MotorType.kFrontLeft.value]
                .set(frontLeftSpeed * maxOutput * frontLeftCorrection);
        motors[MotorType.kFrontRight.value]
                .set(frontRightSpeed * rightSideMultiplier * maxOutput * frontRightCorrection);
        motors[MotorType.kBackLeft.value]
                .set(backLeftSpeed * maxOutput * backLeftCorrection);
        motors[MotorType.kBackRight.value]
                .set(backRightSpeed * rightSideMultiplier * maxOutput * backRightCorrection);
    }

}