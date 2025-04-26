package org.firstinspires.ftc.teamcode.robotcore.opmode.util.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

@TeleOp(group = "2")
public class ExtensionTuning extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        timer.reset();

        int extPosition = extensionSubsystem.getExtensionPosition();
        extensionSubsystem.setExtensionPower(1);

        while (extPosition <= ExtensionSubsystem.MAX_EXT_POSITION && !isStopRequested()) {
            extPosition = extensionSubsystem.getExtensionPosition();

            telemetry.addData("Extension velocity", extPosition / timer.seconds());
            RobotLog.dd(this.getClass().getSimpleName(), "Extension velocity: %s", extPosition / timer.seconds());

            telemetry.update();
        }
        extensionSubsystem.setExtensionPower(0);
    }
}
