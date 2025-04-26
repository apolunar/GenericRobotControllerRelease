package org.firstinspires.ftc.teamcode.calibration.chassis;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

/**
 * NOTE TO SELF:
 * It looks like the MyRobot class may have no use and instead all commands can be defined via CommandOpMode
 * However, MyRobot could migrate to RobotOpMode which automatically does some fancy things with a robot config
 * to run the robot in an easy way!
 */
@TeleOp(group = "Tuning")
@Disabled
public class LateralPushTuner extends OpMode {
    ChassisSubsystem chassisSubsystem;

    @Override
    public void init() {
        chassisSubsystem = new ChassisSubsystem(hardwareMap);

        chassisSubsystem.register();
    }

    int centerTicks;

    @Override
    public void loop() {
        telemetry.addLine("Running");

        CommandScheduler.getInstance().run();

        centerTicks = chassisSubsystem.getCenterOdometer().getPosition();
        telemetry.addData("Current offset center", centerTicks);
    }

    @Override
    public void stop() {
        telemetry.addLine(String.format("Divide %s ticks by the number of inches you pushed it!", centerTicks));
    }
}
