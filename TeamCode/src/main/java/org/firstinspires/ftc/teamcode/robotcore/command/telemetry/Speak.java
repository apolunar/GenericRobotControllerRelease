package org.firstinspires.ftc.teamcode.robotcore.command.telemetry;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class Speak extends CommandBase {
    private final Telemetry telemetry;
    private final String text;

    private boolean firstRun = true;

    @Override
    public void execute() {
        if (firstRun) {
            telemetry.speak(text);
            telemetry.update();
            firstRun = false;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
