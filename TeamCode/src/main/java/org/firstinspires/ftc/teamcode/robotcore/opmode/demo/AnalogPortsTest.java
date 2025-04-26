package org.firstinspires.ftc.teamcode.robotcore.opmode.demo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class AnalogPortsTest extends OpMode {
    AnalogInput[] analogInputs = new AnalogInput[8];
    Servo s;
    @Override
    public void init() {
        for (int i=0;i<analogInputs.length;i++) {
            analogInputs[i] = hardwareMap.get(AnalogInput.class, i + "");
            s = hardwareMap.get(Servo.class, "s");
        }
    }

    @Override
    public void loop() {
        for (int i=0;i<analogInputs.length;i++) {
            telemetry.addData("Port " + i,analogInputs[i].getVoltage());
        }
        s.setPosition(0.5+(gamepad1.left_stick_y * 0.5));
    }
}
