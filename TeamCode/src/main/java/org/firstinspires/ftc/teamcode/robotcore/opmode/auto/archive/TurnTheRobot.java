package org.firstinspires.ftc.teamcode.robotcore.opmode.auto.archive;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robotcore.command.chassis.DriveToPoint;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@Autonomous(group = "Main")
@Disabled
public class TurnTheRobot extends CommandOpMode {
    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        schedule(
                new SequentialCommandGroup(
                        new DriveToPoint(
                                chassisSubsystem,
                                new Pose2d(0, 0, Rotation2d.fromDegrees(90))
                        )
                )
        );
        register(chassisSubsystem);
    }
}
