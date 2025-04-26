package org.firstinspires.ftc.teamcode.demos.concept;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.OdoWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import lombok.RequiredArgsConstructor;

@TeleOp(name = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!", group = "Concept")
@Disabled
public class ConceptTrajectory extends CommandOpMode {

    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        Pose2d start = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d end = new Pose2d(0, 0.1, new Rotation2d(0));

        ArrayList<Translation2d> waypoints = new ArrayList<>();
        waypoints.add(new Translation2d(0, 0.5));

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(0.1, 0.1);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                waypoints,
                end,
                trajectoryConfig
        );

        RamseteController ramseteController = new RamseteController(2.1, 0.8);

        FollowerCommand followerCommand = new FollowerCommand(chassisSubsystem, trajectory, ramseteController, this::getRuntime);

//        followerCommand.whenFinished(() -> {
//            chassisSubsystem.drive(0, 0, 0);
//        });

//        RamseteCommand ramseteCommand = new RamseteCommand(
//                trajectory,
//                chassisSubsystem::getLatestPose,
//                ramseteController,
//                new SimpleMotorFeedforward(
//                        0,0,0
//                ),
//                chassisSubsystem.getDriveKinematics(),
//                chassisSubsystem.getWheelSpeeds(),
//                new PIDController(1, 0, 0),
//                new PIDController(1, 0, 0),
//                chassisSubsystem::drive,
//                chassisSubsystem.getDrive()
//        );

        // schedule the command
        schedule(followerCommand);
    }
}

@RequiredArgsConstructor
class FollowerCommand extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;
    private final Trajectory trajectory;
    private final RamseteController ramseteController;
    private final DoubleSupplier runtime;
    @Override
    public void execute() {
        ChassisSpeeds speeds = ramseteController.calculate(chassisSubsystem.getLatestPose(), trajectory.sample(runtime.getAsDouble()));
        MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds = chassisSubsystem.getDriveKinematics().toWheelSpeeds(speeds);
//        chassisSubsystem.driveWheelSpeeds(mecanumDriveWheelSpeeds);
    }

    @Override
    public boolean isFinished() {
        return ramseteController.atReference();
    }
}