package org.firstinspires.ftc.teamcode.robotcore.command.action;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class WaitSeconds extends CommandBase {
    private final double seconds;

    private ElapsedTime elapsedTime;

    private boolean firstRun = true;

    @Override
    public void execute() {
        if (firstRun) {
            elapsedTime = new ElapsedTime();
            firstRun = false;
        }
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.seconds() >= seconds;
    }
}
