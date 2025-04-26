package org.firstinspires.ftc.teamcode.robotcore.opmode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.demos.MoveLift;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySegment;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.command.lift.SetLift;
import org.firstinspires.ftc.teamcode.robotcore.command.lift.SetLiftClaw;
import org.firstinspires.ftc.teamcode.robotcore.command.action.WaitSeconds;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

@Autonomous
public class TestCommand extends CommandOpMode {
    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);

//        TrajectorySequence trajectorySequence = TrajectorySequence.weaveTrajectorySequence(
//                new TrajectorySegment[]{
//                        new TrajectorySegment(
//                                Rotation2d.fromDegrees(0),
//                                new Translation2d[0],
//                                new Pose2d(ObstacleMap.INCHES_PER_TILE * 2, 0, Rotation2d.fromDegrees(0)),
//                                Rotation2d.fromDegrees(0),
//                                MyRobot.RobotConfig.trajectoryConfig
//                        ),
//                }
//        );
        TrajectorySequence trajectorySequence = TrajectorySequence.weaveTrajectorySequence(
                new TrajectorySegment[]{
                        new TrajectorySegment(
                                Rotation2d.fromDegrees(90),
                                new Translation2d[0],
                                new Pose2d(0, ObstacleMap.INCHES_PER_TILE * 2, Rotation2d.fromDegrees(90)),
                                Rotation2d.fromDegrees(0),
                                AutoConfig.TRAJECTORY_CONFIG
                        ),
                }
        );

//        liftSubsystem.zeroLiftPosition();

        extensionSubsystem.setExtensionPosition(0);

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        double voltage = voltageSensor.getVoltage();
        double voltageOffset = (voltage - 11) / 3;
        RobotLog.dd("TestCommand", "Voltage offset: %s", voltageOffset);

        int newLiftPosition = LiftSubsystem.DELIVERY_POSITION + (int)(30 * voltageOffset);
        RobotLog.dd("TestCommand", "New lift position: %s", newLiftPosition);

        schedule(
                new SequentialCommandGroup(
                        new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION),
                        new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION)
//                        new HoldExtension(extensionSubsystem, 0)
//                    new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, 2000)
//                    new SetArmClawPitch(extensionSubsystem, ExtensionSubsystem.Pitch.HIGH),
//                    new LogRobotLog(telemetry, "Set arm claw pitch"),
//                    new WaitSeconds(1),
//                    new SetArmYawVoltage(extensionSubsystem, 1.22),
//                    new LogRobotLog(telemetry, "Set arm yaw voltage"),
//                    new WaitSeconds(1),
//                    new SetDiffyRoll(extensionSubsystem, 1),
//                    new LogRobotLog(telemetry, "Set diffy"),
//                    new WaitSeconds(1),
//                    new SetDiffyClaw(extensionSubsystem, ExtensionSubsystem.ClawPosition.CLOSED),
//                    new LogRobotLog(telemetry, "Set claw")
//                        new SetDiffyPosition(extensionSubsystem, ExtensionSubsystem.ClawPitch.MID, 0)
//                        new MoveSwing(liftSubsystem, LiftSubsystem.SwingPosition.DELIVERY),
//                        new WaitSeconds(1),
//                        new MoveSwing(liftSubsystem, LiftSubsystem.SwingPosition.INTAKE)
//                        new ResetLift(liftSubsystem),
//                        new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, 2000),
//                        new WaitSeconds(1),
//                        new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION, 2000),
//                        new WaitSeconds(1),
//                        new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, 2000)

//                        new ParallelCommandGroup(
//                                new MoveLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, 2000),
//                                new WaitSeconds(1),
//                                new MoveLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION, 2000)
//                                new FollowTrajectory(
//                                    chassisSubsystem,
//                                    MyRobot.RobotConfig.controller,
//                                    trajectorySequence.get(0)
//                            )
//                        )
//                        new ResetLift(liftSubsystem),
//                        new MoveSwing(liftSubsystem, LiftSubsystem.SwingPosition.DELIVERY)
//                        new WaitSeconds(1),
//                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN),
//                        new WaitSeconds(1),
//                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED),
//                        new WaitSeconds(1),
//                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN),
//                        new WaitSeconds(1),
//                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED),
//                        new WaitSeconds(1),
//                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN)
//                        new ResetLift(liftSubsystem),
//                        new MoveLift(liftSubsystem, -1875, 1600),
//                        new FollowTrajectory(
//                                chassisSubsystem,
//                                MyRobot.RobotConfig.controller,
//                                trajectorySequence.get(0)
//                        )
                )
        );
    }
}
