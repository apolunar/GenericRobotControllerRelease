package org.firstinspires.ftc.teamcode.robotcore.command.action;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;

import java.util.Objects;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class InitAprilTagVision extends CommandBase {
    private final CameraSubsystem cameraSubsystem;
    private final CameraSubsystem.RobotCamera camera;

    @Override
    public void initialize() {
        /*CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(
                        cameraSubsystem.cameras., webcam2)*/

        cameraSubsystem.setupAprilTagPortal(camera);
        RobotLog.dd(this.m_name, "Setup vision portal for TrajectoryToAprilTag");
    }
//    public void switchCamera(CameraSubsystem.Camera activeCamera) {
//        WebcamName cameraName = Objects.requireNonNull(cameraSubsystem.cameras.get(activeCamera)).webcamName;
//        cameraSubsystem.switchActiveCamera(cameraName);
//    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
