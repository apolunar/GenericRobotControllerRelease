package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

import lombok.RequiredArgsConstructor;

/**
 * Completes instantly.
 */
@RequiredArgsConstructor
public class SetArmClawPitch extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;

    private final ExtensionSubsystem.Pitch pitch;

    @Override
    public void execute() {
        extensionSubsystem.setArmClawPitchSync(pitch);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
