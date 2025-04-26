package org.firstinspires.ftc.teamcode.demos.concept;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.demos.SpatulaSubsystem;

@TeleOp(group = "Concept")
@Disabled
public class ConceptTransferSensorColor extends CommandOpMode {
    private SpatulaSubsystem spatulaSubsystem;

    @Override
    public void initialize() {
        spatulaSubsystem = new SpatulaSubsystem(hardwareMap);
    }

    @Override
    public void run() {
        telemetry.addData("Pixels", spatulaSubsystem.getPixels());
    }
}
