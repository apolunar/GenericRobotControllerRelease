package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.util.Range;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class SetExtension extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;

    private final int position;
    private final int targetVelocity;

    @Override
    public void execute() {
        extensionSubsystem.setExtensionPosition(position, targetVelocity);
    }

    @Override
    public boolean isFinished() {
        double extensionPosition = extensionSubsystem.getExtensionPosition();
        RobotLog.dd(this.m_name, "Extension position : %s", extensionPosition);
        boolean complete = Range.valueWithinPercentage(extensionPosition, position, AutoConfig.SUBSYSTEM_ERROR);
        if (complete) {
            RobotLog.dd(this.m_name, "Extension reached target...");
        }
        return complete;
    }

    @Override
    public void end(boolean interrupted) {
        extensionSubsystem.setExtensionPower(0);
    }
}
