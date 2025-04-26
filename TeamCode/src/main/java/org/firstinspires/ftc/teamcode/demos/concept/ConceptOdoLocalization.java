package org.firstinspires.ftc.teamcode.demos.concept;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

//@TeleOp(group = "Concept")
public class ConceptOdoLocalization extends OpMode {
    ChassisSubsystem chassisSubsystem;

    @Override
    public void init() {
        chassisSubsystem = new ChassisSubsystem(hardwareMap);

        chassisSubsystem.register();
    }

    @Override
    public void loop() {
        telemetry.addLine("Running");

        CommandScheduler.getInstance().run();

        telemetry.addData("Current offset", chassisSubsystem.getLatestPose());
    }
}
