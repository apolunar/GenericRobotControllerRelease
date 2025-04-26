package org.firstinspires.ftc.teamcode.robotcore.opmode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.gamepad.MyGamepadButton;
import org.firstinspires.ftc.teamcode.gamepad.MyToggleButtonReader;
import org.firstinspires.ftc.teamcode.robotcore.command.extension.AlignSampleRoll;
import org.firstinspires.ftc.teamcode.robotcore.game.Alliance;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.vision.SampleSampler;

import lombok.RequiredArgsConstructor;

@Config
@RequiredArgsConstructor
public class TeleOpMode extends CommandBase {
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final ChassisSubsystem chassisSubsystem;
    private final LiftSubsystem liftSubsystem;
    private final ExtensionSubsystem extensionSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private final Telemetry telemetry;
    private final Alliance alliance;

    // CONFIG
    private static final int CAMERA_OFFSET = 100;
    public static double MIN_DRIVE_SPEED = 0.2;
    public static double MED_DRIVE_SPEED = 0.5;
    public static double DEFAULT_DRIVE_SPEED = 1;
    public static double INTAKE_POWER = 0.5;

    // STATE
    private final boolean useDriftCorrection = false;
    private final MyToggleButtonReader ascent2Button = new MyToggleButtonReader();
    private final MyToggleButtonReader ascent3Button = new MyToggleButtonReader();
    private final MyToggleButtonReader disableAdaptiveArm = new MyToggleButtonReader();
    private boolean alignSamples = false;
    private final MyGamepadButton switchAdaptiveArm = new MyGamepadButton();

    private final boolean swingChanged = false;
    private boolean swingSwung = false;
    private boolean justSwung = false;
    private boolean clawChanged = false;
    private boolean clawClosed = true;
    private int targetLiftPosition = LiftSubsystem.INTAKE_POSITION;

    // MISC
    private final ElapsedTime armRuntime = new ElapsedTime();
    private final ElapsedTime clawRuntime = new ElapsedTime();
    private final ElapsedTime clawSwingTimer = new ElapsedTime();
    private final ElapsedTime sampleTransferTimer = new ElapsedTime();
    private final ElapsedTime ascent2Timer = new ElapsedTime();
    private final ElapsedTime ascent3Timer = new ElapsedTime();
    private double clawAngle = 0.5;
    private boolean extFailsafe = false;
    private boolean intakePitchDown = false;
    private boolean firstDpadDown = true;

    // HYBRID
    private AlignSampleRoll alignSampleRoll;

    @Override
    public void initialize() {
//        liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
        liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.INTAKE);
        extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.TRANSFER);

        alignSampleRoll = new AlignSampleRoll(extensionSubsystem, cameraSubsystem, telemetry);
        alignSampleRoll.sampleSampler.setSampleVision(alliance == Alliance.RED_ALLIANCE ? SampleSampler.SampleVision.RED : SampleSampler.SampleVision.BLUE);
    }

    private boolean firstRun = true;

    @Override
    public void execute() {
        if (firstRun) {
            firstRun = false;
            alignSampleRoll.schedule();
        }

        // Setting wheel speeds
        double slowSpeedModifier = gamepad1.left_trigger;
        double medSpeedModifier = gamepad1.right_trigger;
        double rotation = gamepad1.right_stick_x;

        double speedMod = Math.max(MIN_DRIVE_SPEED, DEFAULT_DRIVE_SPEED - (medSpeedModifier * (1 - MED_DRIVE_SPEED)) - (slowSpeedModifier * (1 - MIN_DRIVE_SPEED)));
        double targetRotation = rotation * speedMod;

        double lateral = -gamepad1.left_stick_x * speedMod;
        double forward = gamepad1.left_stick_y * speedMod;

        // Extension

        int extensionPosition = extensionSubsystem.getExtensionPosition();

        double extensionPercentage = (double) extensionPosition / ExtensionSubsystem.MAX_EXT_POSITION;

        telemetry.addData("Extension percentage", extensionPercentage * 100);
        telemetry.addData("Extension limit switch", extensionSubsystem.getLimit());

        int variableExtLimit = (int) (ExtensionSubsystem.EXT_LIMIT_POSITION -
                (ExtensionSubsystem.INTAKE_LENGTH * Math.cos(extensionSubsystem.getZeroedYawRadians())));

        // Extension limits and operation
        double extensionSpeed = 1 - ((1-gamepad2.left_trigger) * 0.55);
        if (!gamepad2.b) {
            if (extensionSubsystem.getLimit()) {
                // limit switch pressed
                extensionSubsystem.zeroSlidePosition(); // zero position
                if (gamepad2.left_stick_y < 0) { // only allow outwards motion
                    extensionSubsystem.setExtensionPower(-gamepad2.left_stick_y * extensionSpeed);
                } else if (gamepad2.left_stick_y > 0) {
                    forward += 0.5 * gamepad2.left_stick_y * extensionSpeed;
                }
            } else if (extensionPosition >= variableExtLimit) { // outside limit
                // Extension Limits
                extensionSubsystem.setExtensionPosition(variableExtLimit - 10,1200);
                extFailsafe = true;
            } else if (extensionPosition >= ExtensionSubsystem.MAX_EXT_POSITION) { // hardware limits
                extFailsafe = true;
            }
            // Normal Operation
            if (extFailsafe) {
                if (gamepad2.left_stick_y < 0) {
                    extensionSubsystem.setExtensionPower(0);
                    forward += 0.5 * gamepad2.left_stick_y * extensionSpeed;
                } else {
                    extFailsafe = false;
                }
            } else {
                // normal operation
                extensionSubsystem.setExtensionPower(-gamepad2.left_stick_y * extensionSpeed);
            }

        }

        if (gamepad2.left_stick_y == 0) {
            extensionSubsystem.setExtensionPosition(extensionPosition);
        }

        // Telemetry
        telemetry.addData("Extension position", extensionPosition);
        telemetry.addData("Extension limit", variableExtLimit);
        telemetry.addData("Intake Yaw:", extensionSubsystem.getYawVoltage());
        telemetry.addData("Extension failsafe", extFailsafe);

        // Claw

        // Yaw Control
        if (!gamepad2.x && !gamepad2.b) {
            // Yaw Limits
            if (intakePitchDown &&
                    (extensionSubsystem.getYawVoltage() < ExtensionSubsystem.RIGHT_YAW_LIMIT) &&
                    gamepad2.left_stick_x > 0) {
                extensionSubsystem.driveYaw(0);
                lateral += -gamepad2.left_stick_x * 0.5 * extensionSpeed;
            } else if (intakePitchDown &&
                    (extensionSubsystem.getYawVoltage() > ExtensionSubsystem.LEFT_YAW_LIMIT) &&
                    gamepad2.left_stick_x < 0) {
                extensionSubsystem.driveYaw(0);
                lateral += -gamepad2.left_stick_x * 0.5 * extensionSpeed;
            } else if (extensionPosition >= variableExtLimit * 0.9) {
                extensionSubsystem.driveYaw(-gamepad2.left_stick_x * 0.4 * extensionSpeed);
            } else {
                extensionSubsystem.driveYaw(-gamepad2.left_stick_x * 0.5 * extensionSpeed);
            }
        }

        // === Claw roll

        if (gamepad2.right_stick_y == 0) {
            // Preventing divide by 0
            if (gamepad2.right_stick_x > 0) {
                clawAngle = -Math.PI/2;
            } else if (gamepad2.right_stick_x < 0) {
                clawAngle = Math.PI/2;
            }
        } else if (gamepad2.right_stick_x != 0 || gamepad2.right_stick_y != 0) {
            // Points the claw at the direction the joystick is in
            clawAngle = Math.atan(gamepad2.right_stick_x/gamepad2.right_stick_y);
        }

        if (switchAdaptiveArm.wasJustPressed(gamepad2.left_stick_button)) {
            alignSamples = !alignSamples;
            alignSampleRoll.sampleSampler.setSampleVision(alignSamples ?
                    SampleSampler.SampleVision.YELLOW : (alliance == Alliance.RED_ALLIANCE ? SampleSampler.SampleVision.RED : SampleSampler.SampleVision.BLUE));
        }

        // Only follow joystick if no rect or disabled
        disableAdaptiveArm.update(gamepad2.right_stick_button);
        alignSampleRoll.setEnableRoll(!disableAdaptiveArm.isPressed());
        // || alignSampleRoll.sampleSampler.getLargestRect() == null
        if (disableAdaptiveArm.isPressed()) {
            extensionSubsystem.setClawAngle(clawAngle - extensionSubsystem.getZeroedYawRadians());;
        }

        // === End claw roll

        if (gamepad2.a) {
            intakePitchDown = true;
            extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.MID);
            extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.OPEN);
        } else if (gamepad2.b) {
            if(!extensionSubsystem.getLimit()){
                extensionSubsystem.setExtensionPower(-.75);
            }
            extensionSubsystem.setArmYawVoltage(ExtensionSubsystem.SIDE_DEPOSIT);
        } else if (gamepad2.x) {
            extensionSubsystem.setArmYawVoltage(ExtensionSubsystem.CENTER_YAW);
        } else if (gamepad2.y) {
            intakePitchDown = false;
            extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.HIGH);
        } else if (gamepad2.back) {
            // Sample transfer
            if (sampleTransferTimer.seconds() > 1.75) {
                liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.BASKET);
                liftSubsystem.setLiftPosition(LiftSubsystem.SAMPLE_DELIVERY_POSITION);
                targetLiftPosition = LiftSubsystem.SAMPLE_DELIVERY_POSITION;
            } else if (sampleTransferTimer.seconds() > 1.5) {
                extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.OPEN);
            } else if (sampleTransferTimer.seconds() > 1.25) {
                liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
            } else if (sampleTransferTimer.seconds() > 0.5) {
                liftSubsystem.setLiftPosition(LiftSubsystem.SAMPLE_TRANSFER_POSITION);
                liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.OPEN);
            } else {
                liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
                extensionSubsystem.setArmYawVoltage(ExtensionSubsystem.CENTER_YAW);
                extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.TRANSFER);
                extensionSubsystem.setExtensionPosition(ExtensionSubsystem.SAMPLE_TRANSFER_POSITION);
                liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.PARALLEL);
                liftSubsystem.setLiftPosition(LiftSubsystem.DELIVERY_POSITION);
            }
        }
        if (!gamepad2.back) {
            sampleTransferTimer.reset();
        }

        if (gamepad2.right_bumper) {
            extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.OPEN);
        } else if (gamepad2.right_trigger > 0) {
            if (clawRuntime.milliseconds() > 400) {
                alignSampleRoll.setEnableRoll(true);
                extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.HIGH);
                intakePitchDown = false;
            } else if (clawRuntime.milliseconds() > 250) {
                extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.MID);
            } else if (clawRuntime.milliseconds() > 100) {
                extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.CLOSED);
            } else {
                alignSampleRoll.setEnableRoll(false);
                extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.INTAKE);
//                if (!disableAdaptiveArm.isPressed()) {
//                    extensionSubsystem.setExtensionPosition(extensionPosition - CAMERA_OFFSET);
//                }
            }
        } else if (gamepad2.right_trigger == 0) {
            clawRuntime.reset();
        }
        // Lift

        // Slide control
        telemetry.addData("Arm position", liftSubsystem.getLiftPosition());
        telemetry.addData("Arm limit", liftSubsystem.getLimit());

        // Claw control
        if (gamepad2.left_bumper || gamepad1.right_bumper) {
            if (!clawChanged) {
                if (clawClosed) {
                    liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.OPEN);
                } else {
                    liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
                }
                clawClosed = !clawClosed;
            }
            clawChanged = true;
        } else {
            clawChanged = false;
        }
        // Preset positions
        if (gamepad2.dpad_up || gamepad1.dpad_up) { // Delivery Position
            extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.OPEN);
            liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
            clawClosed = true;
            if (armRuntime.milliseconds() > 600) {
                targetLiftPosition = LiftSubsystem.DELIVERY_POSITION;
                if (liftSubsystem.getLiftPosition() < LiftSubsystem.SHOULDSWING_POSITION) {
                    liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.DELIVERY);
                    swingSwung = true;
                }
            }

        } else if (gamepad1.right_bumper) { // Submersible withdraw
            targetLiftPosition = LiftSubsystem.WITHDRAW_POSITION;
        }
//            else if (gamepad2.back) {
//                liftSubsystem.toggleClaw(LiftSubsystem.ClawPosition.CLOSED);
//                clawClosed = true;
//                liftSubsystem.setLiftPosition(-1000);}
        else if (gamepad2.dpad_down || gamepad1.dpad_down || gamepad2.b) { // Specimen Intake Position
            if (firstDpadDown) {
                if (liftSubsystem.getLimit()) {
                    liftSubsystem.zeroLiftPosition();
                    firstDpadDown = false;
                }
            } else {
                targetLiftPosition = LiftSubsystem.INTAKE_POSITION;
                if (liftSubsystem.getLiftPosition() >= LiftSubsystem.SHOULDSWING_POSITION) {
                    if (!justSwung) {
                        justSwung = true;
                        clawSwingTimer.reset();
                    }
                    liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.INTAKE);
                    if (clawSwingTimer.seconds() > .25) {
                        liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.OPEN);
                        clawClosed = false;
                    } else {
                        liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
                        clawClosed = true;
                    }
                } else {
                    liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
                    justSwung = false;
                    clawClosed = true;
                }
                swingSwung = false;
            }
        }
        if (!gamepad2.dpad_up && !gamepad1.dpad_up) {
            armRuntime.reset();
        }
        if (gamepad2.dpad_left || gamepad2.dpad_right || (firstDpadDown && (gamepad2.dpad_down || gamepad1.dpad_down))) { // Drives normally if not using presets
            if (liftSubsystem.getLimit()) { // Resets slide position if limit switch is pressed
                liftSubsystem.zeroLiftPosition();
                if (gamepad2.dpad_left) {
                    liftSubsystem.setLiftPower(-1);
                } else {
                    liftSubsystem.setLiftPower(0);
                }
            } else {
                if (gamepad2.dpad_right) {
                    liftSubsystem.setLiftPower(0.5);
                } else if (gamepad2.dpad_left) {
                    liftSubsystem.setLiftPower(-1);
                } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
                    liftSubsystem.setLiftPower(1);
                } else {
                    liftSubsystem.setLiftPower(0);
                }
            }
            targetLiftPosition = liftSubsystem.getLiftPosition();
        } else if (!gamepad2.back && !ascent2Button.isPressed()) {
            liftSubsystem.setLiftPosition(targetLiftPosition);
        }

        // Pivot control
//            if (gamepad2.left_trigger > 0 && !swingChanged) {
//                swingSwung = !swingSwung;
//                liftSubsystem.setSwingPosition(swingSwung ? LiftSubsystem.SwingPosition.DELIVERY : LiftSubsystem.SwingPosition.INTAKE);
//                swingChanged = true;
//            } else if (!(gamepad2.left_trigger > 0)) swingChanged = false;

        // Drive
        ascent2Button.update(gamepad1.x);
        ascent3Button.update(gamepad1.y);
        if (!ascent2Button.isPressed()) {
            chassisSubsystem.drive(
                    lateral,
                    forward,
                    targetRotation);
            chassisSubsystem.drivePTO(0.16);
            ascent2Timer.reset();
        } else { // Ascent
            // L3
            if (!ascent3Button.isPressed()) {
                ascent3Timer.reset();
            } else {
                ascent2Timer.reset();
            }
            extensionSubsystem.setExtensionPosition(0);
            if (ascent3Button.isPressed() && !liftSubsystem.getLimit()) {
                chassisSubsystem.getLeftBackMotor().set(0);
                chassisSubsystem.getRightBackMotor().set(0);
                liftSubsystem.setLiftPower(0.75);
                chassisSubsystem.getLeftFrontMotor().set(-1);
                chassisSubsystem.getRightFrontMotor().set(1);
            } else if (ascent3Button.isPressed()) {
                liftSubsystem.setLiftPower(0.3);
                chassisSubsystem.getLeftFrontMotor().set(-0.4);
                chassisSubsystem.getRightFrontMotor().set(0.4);
            }
            if (gamepad1.a) {
                chassisSubsystem.getLeftBackMotor().set(1);
                chassisSubsystem.getRightBackMotor().set(-1);
                chassisSubsystem.getLeftFrontMotor().set(0);
                chassisSubsystem.getRightFrontMotor().set(0);
            } else if (ascent2Timer.seconds() > 3.25){
                chassisSubsystem.getLeftBackMotor().set(0.1);
                chassisSubsystem.getRightBackMotor().set(-0.1);
                chassisSubsystem.getLeftFrontMotor().set(0);
                chassisSubsystem.getRightFrontMotor().set(0);
            } else if (ascent2Timer.seconds() > 2.25) {
                chassisSubsystem.getLeftBackMotor().set(0.8);
                chassisSubsystem.getRightBackMotor().set(-0.8);
                chassisSubsystem.getLeftFrontMotor().set(0.6);
                chassisSubsystem.getRightFrontMotor().set(-0.6);
            } else if (ascent2Timer.seconds() > 1.25) {
                chassisSubsystem.drivePTO(0.5);
            } else if (!ascent3Button.isPressed()) {
                liftSubsystem.setLiftPosition(-2160);
            }

        }
//            chassisSubsystem.periodic();
        telemetry.update();
    }
}
