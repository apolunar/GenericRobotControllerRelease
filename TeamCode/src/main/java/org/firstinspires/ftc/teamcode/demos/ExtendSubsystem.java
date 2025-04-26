package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.logging.LogWrapper;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

import lombok.Getter;

/**
 * Moves the pixel to the backdrop.
 */
@Getter
@Config
public class ExtendSubsystem extends SubsystemBase implements RobotSubsystem {
    // CONFIG
    public static int MAX_SLIDE_TICKS = 2800;
    private static final String LEFT_SLIDE_NAME  = "slideLeft";
    private static final String RIGHT_SLIDE_NAME = "slideRight";
    private static final String LEFT_LIMIT_NAME  = "limitL";
    private static final String RIGHT_LIMIT_NAME = "limitR";

    // HARDWARE
    private final DcMotorEx slideLeft;
    private final DcMotorEx slideRight;
    private final TouchSensor limitL;
    private final TouchSensor limitR;

    // CONFIG
    private final PController armController = new PController(0.8);

    public ExtendSubsystem(DcMotorEx slideLeft, DcMotorEx slideRight, TouchSensor limitLeft, TouchSensor limitRight) {
        this.slideLeft  = slideLeft;
        this.slideRight = slideRight;
        this.limitL = limitLeft;
        this.limitR = limitRight;

        // [!] Motors must run at same velocity or the slide subsystem **will** break
        this.slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public ExtendSubsystem(HardwareMap hMap) {
        this(hMap.get(DcMotorEx.class, LEFT_SLIDE_NAME), hMap.get(DcMotorEx.class, RIGHT_SLIDE_NAME), hMap.get(TouchSensor.class, LEFT_LIMIT_NAME), hMap.get(TouchSensor.class, RIGHT_LIMIT_NAME));
    }

    public int getSlideTicks() {
        return (-slideLeft.getCurrentPosition() + slideRight.getCurrentPosition()) / 2;
    }

    public void zeroSlidePosition() {
        this.slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setOutput(double output, boolean override) {
        //Limits should be backwards
        this.slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LogWrapper.log(this.m_name, "Slide ticks : %s", getSlideTicks());
        if ((( (getSlideTicks() <= MAX_SLIDE_TICKS) || (output < 0)) && (!limitPressed() || (output > 0))) || override) {
            LogWrapper.log(this.m_name, "Move slide");
            slideLeft.setPower(-output);
            slideRight.setPower(output);
        }
        else {
            LogWrapper.log(this.m_name, "Not move slide");
            slideLeft.setPower(0);
            slideRight.setPower(0);
        }
    }

    public void setSlidePower(double output) {
        //Limits should be backwards
        this.slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideLeft.setPower(-output);
        slideRight.setPower(output);
    }

    public boolean limitPressed() {
        return (!limitR.isPressed() || !limitL.isPressed()); // NOTE: limitR and limitL are reversed boolean-wise due to wiring D:
    }

    public void setOutput(double power) {
        setOutput(power, false);
    }

    public void setVelocity(double velocity) {
        slideLeft.setVelocity(velocity);
        slideRight.setVelocity(velocity);
    }

    public void setTargetTicks(int position, double velocity, boolean override) {
        if ((position >= MAX_SLIDE_TICKS || position < 0) && !override) return;

//        double output = armController.calculate(getSlideTicks(), position);
//
//        slideLeft.setPower(-output);
//        slideRight.setPower(output);

        slideLeft.setTargetPosition(-position);
        slideRight.setTargetPosition(position);

        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideLeft.setVelocity(velocity);
        slideRight.setVelocity(velocity);
    }

    public void setTargetTicks(int position) {
        setTargetTicks(position, 800, false);
    }
}
