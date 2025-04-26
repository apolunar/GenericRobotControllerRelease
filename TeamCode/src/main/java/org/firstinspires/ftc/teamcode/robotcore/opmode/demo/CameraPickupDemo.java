package org.firstinspires.ftc.teamcode.robotcore.opmode.demo;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.command.chassis.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robotcore.command.extension.AlignSampleRoll;
import org.firstinspires.ftc.teamcode.robotcore.command.extension.PickupSample;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.vision.SampleSampler;

import java.util.ArrayList;

@TeleOp(group = "2")
public class CameraPickupDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);
        CameraSubsystem cameraSubsystem = new CameraSubsystem(hardwareMap);

//        PickupSample pickupSample = new PickupSample(extensionSubsystem, cameraSubsystem, telemetry);
//        pickupSample.schedule();
        AlignSampleRoll alignSampleRoll = new AlignSampleRoll(extensionSubsystem, cameraSubsystem, telemetry);
        alignSampleRoll.schedule();
        alignSampleRoll.sampleSampler.setSampleVision(SampleSampler.SampleVision.YELLOW);

        extensionSubsystem.setArmPitch(ExtensionSubsystem.ArmPitch.MID);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetry.update();
        }
    }
}
