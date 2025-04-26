package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotConfig;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.vision.AngleStabilizer;
import org.firstinspires.ftc.teamcode.robotcore.vision.SampleSampler;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import lombok.RequiredArgsConstructor;

@Config
public class PickupSample extends SequentialCommandGroup {
    private final ExtensionSubsystem extensionSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final Telemetry telemetry;


    public PickupSample(ExtensionSubsystem extensionSubsystem, CameraSubsystem cameraSubsystem, LimelightSubsystem limelightSubsystem, Telemetry telemetry) {
        this.extensionSubsystem = extensionSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.telemetry = telemetry;

        addCommands(

        );
        addRequirements(extensionSubsystem, cameraSubsystem, limelightSubsystem);
    }

    private boolean firstRun = true;
//    public static double kP = 0.0001;
//    public static double kI = 0.;
//    public static double kD = 0.;
//    private PIDController visionController = new PIDController(kP, kI, kD);
//    public static Point targetImagePoint = new Point(CameraSubsystem.RobotCamera.DIFFY_CAM.width / 2., CameraSubsystem.RobotCamera.DIFFY_CAM.height / 2.);

    @Override
    public void execute() {
        if (firstRun) {
            firstRun = false;
        }


    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: Null object reference????
//        cameraSubsystem.visionPortal.setProcessorEnabled(sampleSampler, false);
    }
}
