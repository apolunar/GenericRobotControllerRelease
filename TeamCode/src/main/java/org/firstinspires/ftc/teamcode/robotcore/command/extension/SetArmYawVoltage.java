package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

import lombok.RequiredArgsConstructor;

/**
 * Waits for yaw.
 */
@RequiredArgsConstructor
public class SetArmYawVoltage extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;

    private final double armYawVoltage;

    @Override
    public void execute() {
        extensionSubsystem.setArmYawVoltage(armYawVoltage);
    }

    @Override
    public boolean isFinished() {
        RobotLog.dd(this.m_name, "Arm yaw voltage : %s", extensionSubsystem.getYawVoltage());
//        return Range.valueWithinPercentage(extensionSubsystem.getYawVoltage(), armYawVoltage, MyRobot.RobotConfig.SUBSYSTEM_ERROR);
        return false;
    }
}
