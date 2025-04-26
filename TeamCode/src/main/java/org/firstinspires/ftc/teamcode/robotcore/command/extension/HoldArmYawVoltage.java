package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class HoldArmYawVoltage extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;

    private final double armYawVoltage;

    @Override
    public void execute() {
        extensionSubsystem.setArmYawVoltage(armYawVoltage);
    }

    @Override
    public boolean isFinished() {
//        RobotLog.dd(this.m_name, "Arm yaw voltage : %s", extensionSubsystem.getYawVoltage());
        return false;
    }
}
