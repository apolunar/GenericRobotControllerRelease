package org.firstinspires.ftc.teamcode.robotcore.opmode.auto.archive;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotStateStore;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

//@TeleOp
public class ResetRobotStateTracker extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        RobotStateStore.reset();

        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        chassisSubsystem.getHolonomicOdometry().updatePose(new Pose2d(0, 0, new Rotation2d(0)));
    }
}
