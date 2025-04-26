package org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;


/**
 * The Big Bad Pixel eater. Moves pixels to the spatula.
 */
@Config
public class LiftSubsystem extends SubsystemBase implements RobotSubsystem {
    // CONFIG
    private static final String SLIDE_NAME = "armSlide";
    private static final String SWING_SERVO_NAME = "swing";
    private static final String CLAW_NAME = "claw";
    private static final String LIMIT_NAME = "armLimit";

    public static int INTAKE_POSITION = -235;
    public static int DELIVERY_POSITION = -1525;
    public static int WITHDRAW_POSITION = -1625;
    public static int SHOULDSWING_POSITION = -700;
    public static int SAMPLE_TRANSFER_POSITION = -379;
    public static int SAMPLE_DELIVERY_POSITION = -2412;

    public static int MAX_SLIDE_POSITION = 3000;

    public enum ClawPosition {
        OPEN(0.24),
        CLOSED(0.49);

        public final double position;
        ClawPosition(double position) {
            this.position = position;
        }
    }

    public enum SwingPosition {
        INTAKE(0.985),
        PARALLEL(0.37),
        DELIVERY(0.33),
        BASKET(0.12);
        double position;
        SwingPosition(double position) {
            this.position = position;
        }
    }

    // HARDWARE
    private final DcMotorEx liftSlide;
    private final Servo swing;
    private final Servo claw;
    private final TouchSensor limit;

    public static double lift_kP = 0.01;
    public static double lift_kD = 0.0;
    public static double lift_kI = 0.2;
    public static double lift_feedforward = 0.0;
    private final PIDFController liftController = new PIDFController(lift_kP, lift_kI, lift_kD, lift_feedforward);

    // MISC
    private ClawPosition internalClawPosition;

    public LiftSubsystem(DcMotorEx liftSlide, Servo swing, Servo claw, TouchSensor limit) {
        this.liftSlide = liftSlide;
        this.swing = swing;
        this.claw = claw;
        this.limit = limit;

        RobotLog.dd(this.m_name, "Lift start position: %s", liftSlide.getCurrentPosition());

        internalClawPosition = ClawPosition.CLOSED;
        this.liftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.liftSlide.setCurrentAlert(15, CurrentUnit.AMPS);
        zeroLiftPosition();
    }

    public LiftSubsystem(HardwareMap hMap) {
        this(hMap.get(DcMotorEx.class, SLIDE_NAME),
                hMap.get(Servo.class, SWING_SERVO_NAME),
                hMap.get(Servo.class, CLAW_NAME),
                hMap.get(TouchSensor.class, LIMIT_NAME)
        );
    }

    public double getLiftCurrent() {
        return liftSlide.getCurrent(CurrentUnit.AMPS);
    }

    // Slide
    public void setLiftPower(double power) {
        liftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftSlide.setPower(power);
    }

    public int getLiftPosition() {
        return liftSlide.getCurrentPosition();
    }

    public void zeroLiftPosition() {
        liftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setLiftPosition(int position, int power) {
        liftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double liftPosition = getLiftPosition();
        double output = liftController.calculate(liftPosition, position);
        liftSlide.setPower(output);
//        RobotLog.dd(this.m_name, "New lift position: %s, current: %s, and output: %s", position, liftPosition, output);
//        liftSlide.setTargetPosition(position);
//        liftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        liftSlide.setVelocity(velocity);
    }


    /**
     * Go to slide position at velocity default 1400
     *
     * @param position
     */
    public void setLiftPosition(int position) {
        setLiftPosition(position, 1400);
    }

    public boolean getLimit() {
        return !limit.isPressed();
    }

    // Claw
    public void setClaw(double position) {
        claw.setPosition(position);
    }

    public void setClawPosition(ClawPosition newClawPosition) {
        internalClawPosition = newClawPosition;
        setClaw(newClawPosition.position);
    }

    public double getInternalClawPosition() {
        return claw.getPosition();
    }

    // Swing
    public void setSwing(double position) {
        if (internalClawPosition == ClawPosition.CLOSED) { // can only swing if claw is closed
            swing.setPosition(position);
        }
    }

    public void setSwingPosition(SwingPosition swing) {
        setSwing(swing.position);
    }
}
