package org.firstinspires.ftc.teamcode.robotcore.opmode.auto.tests;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TestMotorCurrent extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
//        MotorEx motor = new MotorEx(hardwareMap, "motor");
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        telemetry.addData("", motor.getCurrent(CurrentUnit.AMPS));
    }
}
