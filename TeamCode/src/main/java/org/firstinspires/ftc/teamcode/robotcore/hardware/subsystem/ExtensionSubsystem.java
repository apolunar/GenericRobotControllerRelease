package org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.util.Range;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

import lombok.Getter;

@Config
public class ExtensionSubsystem extends SubsystemBase implements RobotSubsystem {
    // CONFIG
    private static final String INTAKE_SLIDE_NAME = "intakeSlide";
    private static final String CLAW_SERVO_NAME  = "intake claw";
    private static final String LEFT_DIFFY_SERVO_NAME = "diffy l";
    private static final String RIGHT_DIFFY_SERVO_NAME = "diffy r";
    private static final String YAW_SERVO_NAME = "yaw";
    private static final String YAW_ENCODER_NAME = "yaw encoder";
    private static final String PITCH_SERVO_NAME = "pitch";
    private static final String LIMIT_NAME = "extLimit";

    // TODO: Vary based on intake position

    // HARDWARE
    private final DcMotorEx extensionSlide;
    private final Servo clawServo;
    private final Servo lDiffy;
    private final Servo rDiffy;
    private final CRServo yawServo;
    private final AnalogInput yawEncoder;
    private final Servo pitchServo;
    private final TouchSensor limit;

    // Config
    // Limits
    public static int MAX_EXT_POSITION = 1070;
    public static int EXT_LIMIT_POSITION = 1150;
    public static int SAMPLE_TRANSFER_POSITION = 20;
    public static int INTAKE_LENGTH = 450;
    private static final double SIDE_DELIVERY = 0.61;

    public static double extension_kP = 0.01;
    public static double extension_kD = 0.0;
    public static double extension_kI = 1e-1;
    public static double extension_feedforward = 0.0;
    private final PIDFController extensionController = new PIDFController(extension_kP, extension_kI, extension_kD, extension_feedforward);

    // Utils
    final double RADIANS_PER_VOLTAGE = (Math.PI*2)/3.3;

    // Software
    @Getter
    double internalClawRoll = 0.5;
    @Getter
    double internalClawPitch = 0.5;

    public enum ClawPosition {
        OPEN(0.17),
        CLOSED(0.67);

        private final double position;
        ClawPosition(double position) {
            this.position = position;
        }
    }

    public enum ClawPitch {
        INTAKE(0.5),
        MID(0.5),
        TRANSFER(0.0);

        public final double position;
        ClawPitch(double position) {
            this.position = position;
        }
    }

    public enum ArmPitch {
        INTAKE(0.05),
        MID(0.13),
        HIGH(0.18),
        ADAPTIVE(0.25),
        TRANSFER(0.63);

        public final double position;
        ArmPitch(double position) {
            this.position = position;
        }
    }

    public static final double LEFT_YAW_LIMIT = 1.688;
    public static final double RIGHT_YAW_LIMIT = 0.913;
    public static final double CENTER_YAW = 1.295;
    public static final double SIDE_DEPOSIT = 0.315;
    public enum ArmYaw {
        LEFT(LEFT_YAW_LIMIT),
        RIGHT(RIGHT_YAW_LIMIT),
        CENTER(CENTER_YAW);

        public final double position;
        ArmYaw(double position) {
            this.position = position;
        }
    }

    // TODO: This feels very wrong
    private static final ElapsedTime spTimer = new ElapsedTime();
    public static double yaw_kP = 0.5;
    public static double yaw_kD = 0.1;
    public static double yaw_kI = 0.0;
    private final PIDController yawController = new PIDController(yaw_kP, yaw_kI, yaw_kD);

    int latestExtensionPosition = 0;

    // TODO: Horrible enum for single use-case
    public enum Pitch {
        INTAKE(ClawPitch.INTAKE, ArmPitch.INTAKE),
        MID(ClawPitch.MID, ArmPitch.MID),
        ADAPTIVE(ClawPitch.TRANSFER, ArmPitch.ADAPTIVE),
        HIGH(ClawPitch.TRANSFER, ArmPitch.HIGH),
        TRANSFER(ClawPitch.TRANSFER, ArmPitch.TRANSFER);
        public final ClawPitch clawPitch;
        public final ArmPitch armPitch;
        Pitch(ClawPitch clawPitch, ArmPitch armPitch) {
            this.clawPitch = clawPitch;
            this.armPitch = armPitch;
        }
    }

    public ExtensionSubsystem(DcMotorEx extensionSlide,
                              Servo claw,
                              Servo lDiffy,
                              Servo rDiffy,
                              CRServo yawServo,
                              AnalogInput yawEncoder,
                              Servo pitchServo,
                              TouchSensor limit) {
        this.extensionSlide = extensionSlide;
        this.clawServo = claw;
        this.lDiffy = lDiffy;
        this.rDiffy = rDiffy;
        this.yawServo = yawServo;
        this.yawEncoder = yawEncoder;
        this.pitchServo = pitchServo;
        this.limit = limit;

        this.extensionSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.extensionSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public ExtensionSubsystem(HardwareMap hMap) {
        this(hMap.get(DcMotorEx.class, INTAKE_SLIDE_NAME),
                hMap.get(Servo.class, CLAW_SERVO_NAME),
                hMap.get(Servo.class, LEFT_DIFFY_SERVO_NAME),
                hMap.get(Servo.class, RIGHT_DIFFY_SERVO_NAME),
                hMap.get(CRServo.class, YAW_SERVO_NAME),
                hMap.get(AnalogInput.class, YAW_ENCODER_NAME),
                hMap.get(Servo.class, PITCH_SERVO_NAME),
                hMap.get(TouchSensor.class, LIMIT_NAME));
    }

    @Override
    public void periodic() {
        latestExtensionPosition = extensionSlide.getCurrentPosition();
    }

    public void setExtensionPower(double power) {
        extensionSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionSlide.setPower(power);
    }

    public void zeroSlidePosition() {
        extensionSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setExtensionPosition(int position, int velocity) {
        extensionSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double extensionPosition = getExtensionPosition();
        double output = extensionController.calculate(extensionPosition, position);
        extensionSlide.setPower(output);
//        RobotLog.dd(this.m_name, "New extension position: %s, current: %s, and output: %s", position, extensionPosition, output);
    }

    public void holdExtensionPosition() {
        extensionSlide.setTargetPosition(getExtensionPosition());
        extensionSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extensionSlide.setVelocity(800);
    }

    /**
     * Go to slide position at velocity default 800
     * @param position
     */
    public void setExtensionPosition(int position) {
        setExtensionPosition(position, 800);
    }

    public int getExtensionPosition() {
        return latestExtensionPosition;
    }

    public int getVariableExtLimit() {
        return (int) (ExtensionSubsystem.EXT_LIMIT_POSITION -
                (ExtensionSubsystem.INTAKE_LENGTH * Math.cos(getZeroedYawRadians())));
    }

    public boolean getLimit() { return !limit.isPressed(); }

    /**
     * Sets claw rotation and position to the variables
     */
    public void setClawRollPitch(double clawRoll, double clawPitch) {
        internalClawRoll = clawRoll;
        internalClawPitch = clawPitch;
        lDiffy.setPosition((clawRoll + clawPitch) / 2);
        rDiffy.setPosition((clawRoll + (1 - clawPitch)) / 2);
    }

    public void setClawRollPitch(double clawRoll, ClawPitch clawPitch) {
        setClawRollPitch(clawRoll, clawPitch.position);
    }

    public static double wrapAngle(double angle) {
        angle = (angle + Math.PI) % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle - Math.PI;
    }

    public void setClawRoll(double clawRoll) {
        internalClawRoll = clawRoll;
        setClawRollPitch(clawRoll, internalClawPitch);
    }

    public void setClawPitch(double clawPitch) {
        internalClawPitch = clawPitch;
        setClawRollPitch(internalClawRoll, clawPitch);
    }

    /**
     * Drives the yaw to a set voltage between 0 and 3.3 with PID
     * @param voltage
     */
    public void setArmYawVoltage(double voltage) {
        double targetRadians = getYawRadians(voltage);
        double currentRadians = getYawRadians(getYawVoltage());
//        double targetRadians = voltage;
//        double currentRadians = getYawVoltage();
        double error = targetRadians - currentRadians;
//        if (error > 0) {
//            yawServo.setPower(Math.min(1.0, error));
//        } else {
//            yawServo.setPower(Math.max(-1.0, error));
//        }
            // 2PI/3.3 * 180/PI * .005 = .5 degrees
        double output = 0;
        double finalPower = 0;
        if (Math.abs(Math.toDegrees(error)) > .5) {
            spTimer.reset();
             output = -yawController.calculate(currentRadians, targetRadians);
             finalPower = Math.abs(output) > 1 ? 1 * Math.signum(output) : output;
            yawServo.setPower(finalPower);
        } else if (spTimer.milliseconds() > 100) {
            yawController.reset();
            RobotLog.dd(this.m_name, "Reset yaw controller: " + spTimer);
        }
        yawServo.setPower(-output);
////        RobotLog.dd(this.m_name, "Yaw target voltage: %s, voltage: %s, error: %s, and output: %s", voltage, getYawVoltage(), error, output);
//        RobotLog.dd(this.m_name, "Yaw target deg: %s, current deg: %s, error: %s, and output: %.3f/%.3f", Math.toDegrees(targetRadians), Math.toDegrees(currentRadians), Math.toDegrees(error), output, finalPower);
    }

    /**
     * Set arm yaw to position between left limit and right limit.
     * @param position between 0 and 1
     */
    public void setArmYaw(double position) {
        setArmYawVoltage(Range.lerp(RIGHT_YAW_LIMIT, LEFT_YAW_LIMIT, position));
    }

    /**
     * Returns yaw reading
     */
    public double getYawVoltage() {
        return yawEncoder.getVoltage();
    }

    /**
     * Runs the yaw with power
     * @param power
     */
    public void driveYaw(double power) {
        //if (leftLimit < yawEncoder.getVoltage() && yawEncoder.getVoltage() < rightLimit) {
        yawServo.setPower(power);
        //}
    }

    public double getYawRadians(double voltage) {
        return voltage * RADIANS_PER_VOLTAGE;
    }

    /**
     * Returns how far yaw is from the center in radians
     */
    public double getZeroedYawRadians() {
        if (getYawVoltage() != 0) {
            return (yawEncoder.getVoltage() - CENTER_YAW) * RADIANS_PER_VOLTAGE;
        } else {
            return 0;
        }
    }

    /**
     * Moves the intake and yaw to a specific position (theta - yaw angle, x - distance from robot)
     * @param theta radians
     * @param x ticks
     */
    public void setIntakePosition(int theta, int x, int velocity) {
        double angle = Math.asin((double) theta / INTAKE_LENGTH);
        setExtensionPosition((int) (x - (Math.cos(angle)) * INTAKE_LENGTH), velocity);
        setArmYawVoltage((angle / RADIANS_PER_VOLTAGE) + CENTER_YAW);
    }

    public void setIntakePosition(int theta, int x) {
        setIntakePosition(theta, x, 800);
    }

    /**
     * Sets claw pitch to position
     * @param position
     */
    public void setArmPitchServo(double position) { pitchServo.setPosition(position); }

    public void setArmPitch(ArmPitch pitch) {
        setArmPitchServo(pitch.position);
    }

    public void setClawPitch(ClawPitch pitch) {
        setClawPitch(pitch.position);
    }

    /**
     * Sets the arm and claw pitch to a preset
     * @param pitch
     */
    public void setArmClawPitchSync(Pitch pitch) {
        setArmPitch(pitch.armPitch);
        setClawPitch(pitch.clawPitch);
    }

    /**
     * Opens or closes the claw
     * @param clawPosition
     */
    public void setClawPosition(ClawPosition clawPosition) {
        clawServo.setPosition(clawPosition.position);
    }

    /**
     * @param angle in radians
     * @return
     */
    public double angleToClawValue(double angle) {
        // Points the claw at the direction the joystick is in
        // aTan to find the angle the joystick is at
        // Divide by pi to get a value from -0.5 to 0.5
        // The claw roll is a value from -0.1 to 1.1, giving it a range of 1.2
        // Add 0.5 to get a value from -0.1 to 1.1
        return angle * 1.2 / Math.PI + 0.5;
    }

    /**
     * @param angle in radians
     * @return
     */
    public double angleToClawRoll(double angle) {
        return valueToClawRoll(angleToClawValue(angle));
    }

    public double valueToClawRoll(double value) {
        // Mod to prevent the claw from going under or over the range
        return (value + 0.1) % 1.2 - 0.1;
    }

    /**
     * @param angle in radians
     */
    public void setClawAngle(double angle) {
        // Must be at center pitch for roll
        setClawRoll(internalClawPitch != 0.5 ? valueToClawRoll(0.5) : angleToClawRoll(wrapAngle(angle)));
    }
}
