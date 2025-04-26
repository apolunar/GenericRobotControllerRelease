package org.firstinspires.ftc.teamcode.robotcore.command.chassis;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AlignBestSample extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
