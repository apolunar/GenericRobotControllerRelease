package org.firstinspires.ftc.teamcode.robotcore.command.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class LogRobotLog extends CommandBase {
    private final Telemetry telemetry;
    private final String text;

    private boolean firstRun = true;

    @Override
    public void execute() {
        if (firstRun) {
            RobotLog.dd(this.m_name, text);
            firstRun = false;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
