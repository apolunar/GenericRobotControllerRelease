package org.firstinspires.ftc.teamcode.robotcore.tele.input;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.tele.Actions;

public class ArmPickupInput extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;
    private final LiftSubsystem liftSubsystem;
    private final Telemetry telemetry;
    private final Actions actions;

    private final ElapsedTime clawRuntime = new ElapsedTime();
    private double clawRollAngle;
    private double clawRoll = 0.5;

    public ArmPickupInput(ExtensionSubsystem extensionSubsystem, LiftSubsystem liftSubsystem, Telemetry telemetry, Actions actions) {
        this.extensionSubsystem = extensionSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.telemetry = telemetry;
        this.actions = actions;
        addRequirements(extensionSubsystem);
    }

    @Override
    public void execute() {
        if (actions.intakeButton.wasJustPressed()) {
            clawRuntime.reset();
        }

        if (actions.release) {
            extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.OPEN);
        }
        else if (actions.intakeButton.pressed) {
            if (clawRuntime.milliseconds() > 300) {
                extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.HIGH);
            } else if (clawRuntime.milliseconds() > 250) {
                extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.MID);
            } else if (clawRuntime.milliseconds() > 100) {
                extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.CLOSED);
            } else {
                extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.INTAKE);
            }
        } // Intake

        // Claw roll control
//        if (gamepad2.right_stick_y == 0) {
//            // Preventing divide by 0
//            if (gamepad2.right_stick_x > 0) {
//                clawRotation = -0.1;
//            } else if (gamepad2.right_stick_x < 0) {
//                clawRotation = 1.1;
//            }
//        } else
        if (actions.diffyX != 0 || actions.diffyY != 0) {
            // Points the claw at the direction the joystick is in
            // aTan to find the angle the joystick is at
            // Divide by pi to get a value from -0.5 to 0.5
            // The claw roll is a value from -0.1 to 1.1, giving it a range of 1.2
            // Add 0.5 to get a value from -0.1 to 1.1
            clawRollAngle = Math.atan(actions.diffyX / actions.diffyY);
            clawRoll = extensionSubsystem.angleToClawRoll(clawRollAngle);
        }
//        // Robot-oriented claw roll
//        double finalClawRotation = clawRotation; // - (extensionSubsystem.getZeroedYawRadians()*1.2/Math.PI);
//        // Failsafe for raising the extension pitch
//        if (extensionSubsystem.getInternalClawPitch() != 0.5) {
//            finalClawRotation = 0.5;
//        }
        extensionSubsystem.setClawRoll(clawRoll);
        telemetry.addData("Claw roll angle:", clawRollAngle);
        telemetry.addData("Claw roll (center 0.5, +/-1.2):", clawRoll);
    }
}
