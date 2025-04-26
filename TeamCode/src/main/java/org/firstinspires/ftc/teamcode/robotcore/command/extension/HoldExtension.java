package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class HoldExtension extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;

    private final int position;

    @Override
    public void execute() {
        extensionSubsystem.setExtensionPosition(position);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        extensionSubsystem.setExtensionPower(0);
    }
}
