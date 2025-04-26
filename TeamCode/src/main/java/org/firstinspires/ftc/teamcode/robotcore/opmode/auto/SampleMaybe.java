package org.firstinspires.ftc.teamcode.robotcore.opmode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySegment;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.command.action.WaitSeconds;
import org.firstinspires.ftc.teamcode.robotcore.command.chassis.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robotcore.command.extension.HoldExtension;
import org.firstinspires.ftc.teamcode.robotcore.command.lift.SetLift;
import org.firstinspires.ftc.teamcode.robotcore.command.lift.SetLiftClaw;
import org.firstinspires.ftc.teamcode.robotcore.command.telemetry.LogRobotLog;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

@Autonomous
@Disabled
public class SampleMaybe extends CommandOpMode {
    private static final int INTAKE_OFFSET = 50;
    public static int SLIDE_VEL = 2000;

    public static int SLOW_VEL = 45;
    public static int SLOW_ACCEL = 60;
    public static TrajectoryConfig SLOW_TRAJECTORY_CONFIG = new TrajectoryConfig(SLOW_VEL, SLOW_ACCEL);

    // Different forward constraints to lateral constraints
    public static int FORWARD_VEL = AutoConfig.VEL;
    public static int FORWARD_ACCEL = AutoConfig.ACCEL;
    public static TrajectoryConfig FORWARD_TRAJECTORY_CONFIG = new TrajectoryConfig(FORWARD_VEL, FORWARD_ACCEL);

    public static double CLAW_CLOSE_TIME = 0.1;

    public static Pose2d PUSHER_POSE_TOLERANCE = new Pose2d(5.0, 5.0, Rotation2d.fromDegrees(2));
    public static Pose2d PRECISE_POSE_TOLERANCE = new Pose2d(1.5, 1.5, Rotation2d.fromDegrees(2));

    /**
     * Utility to generate sample Y
     * TODO: Seems a bit unnecessary but whatever
     *
     * @param sample
     * @return the Y position of the sample
     */
    private static double sampleY(int sample) {
        return ObstacleMap.INCHES_PER_TILE + 8 * (sample + 1);
    }

    public static TrajectorySegment samplePusher(int sample) {
        double lineupY = sampleY(sample) - 10 + sample * 2.2;
        double lineupX = sample == 0 ? 2.4 : (sample == 1 ? 2. : 2.05);
        return new TrajectorySegment( // to the sample
                Rotation2d.fromDegrees(sample == 0 ? 150 : 0),
                new Translation2d[]{
                        new Translation2d(ObstacleMap.INCHES_PER_TILE + 5, sample == 0 ? 21 : lineupY),
                        new Translation2d(ObstacleMap.INCHES_PER_TILE * lineupX,
                                sample == 0? ObstacleMap.INCHES_PER_TILE - 3 : sample == 2 ? lineupY - 2: lineupY),
                        new Translation2d(ObstacleMap.INCHES_PER_TILE * lineupX, sampleY(sample))
                },
                new Pose2d(15, sampleY(sample), Rotation2d.fromDegrees(-180)),
                Rotation2d.fromDegrees(0),
                FORWARD_TRAJECTORY_CONFIG
        );
    }

    /**
     * @param specimen
     * @return TrajectorySegment of specimen collection, accounting for odometer error by the specimen we're currently doing (TODO: THIS IS BAD)
     * Instead, it would be ideal if we could add sensor fusion/kalman filter to not have to offset trajectories based on experimental accumulated error.
     */
    private static TrajectorySegment specimenCollection(int specimen) {
        return new TrajectorySegment( // to the loading area
                Rotation2d.fromDegrees(-180),
                new Translation2d[0],
                new Pose2d(-0.5 - specimen / 6., sampleY(0) - 9, Rotation2d.fromDegrees(-180)),
                Rotation2d.fromDegrees(0),
                SLOW_TRAJECTORY_CONFIG
        );
    }

    private static TrajectorySegment backToSpecimenCollection() {
        return new TrajectorySegment(
                Rotation2d.fromDegrees(-180 - 40),
                new Translation2d[0],
                new Pose2d(8, sampleY(0) - 9, Rotation2d.fromDegrees(-180)),
                Rotation2d.fromDegrees(0),
                AutoConfig.TRAJECTORY_CONFIG
        );
    }

    private static TrajectorySegment lebron(int sample) {
        double yOffset = 1.5 * sample;
        return new TrajectorySegment( // drive forward
                Rotation2d.fromDegrees(sample == 0 ? -90 : -180),
                new Translation2d[]{
                        new Translation2d(5, ObstacleMap.INCHES_PER_TILE)
                },
                // NOTE: Do not change the `+ 5` IT WILL CAUSE A COMPLETELY UNEXPLAINED RUNTIME ERROR
                // something fails with the trajectory generation
                new Pose2d(ObstacleMap.INCHES_PER_TILE + (sample == 0 ? 10. : 10.5) + sample / 3., -10 - yOffset, Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(-45),
                sample == 0 ? SLOW_TRAJECTORY_CONFIG : AutoConfig.TRAJECTORY_CONFIG
        );
    }

    public static TrajectorySegment parkTrajectory = new TrajectorySegment(
            Rotation2d.fromDegrees(-180),
            new Translation2d[0],
            new Pose2d(5, sampleY(0), Rotation2d.fromDegrees(-135)),
            Rotation2d.fromDegrees(0),
            AutoConfig.TRAJECTORY_CONFIG
    ); // park

    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);

//        extensionSubsystem.setExtensionPosition(0);
//        liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.DELIVERY); // outside of 18
        liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);

        // PRE-FLIGHT CHECKS
        // TODO: Include as datastructure of failure modes
        if (!liftSubsystem.getLimit()) {
            telemetry.speak("Limit switch not pressed!");
            telemetry.addLine("Limit switch not pressed!");
        }

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        double voltage = voltageSensor.getVoltage();
        if (voltage < 12.5) {
            sleep(2000);
            telemetry.speak("Voltage too low!");
            telemetry.addLine("Voltage too low!");
        }
//        double voltageOffset = (voltage - 12) / 14;
//        telemetry.addLine(String.format("Voltage offset: %s", voltageOffset));

        telemetry.update();

        TrajectorySegment[] trajectorySegments = new TrajectorySegment[]{
                lebron(0),

        };

        // TODO: Functions should be more like macros so that the same traj doesn't keep needing to be regenerated but whatever...
        TrajectorySequence trajectorySequence = TrajectorySequence.weaveTrajectorySequence(trajectorySegments);

        RobotLog.dd(this.getClass().getSimpleName(), "Preparing to execute %s trajectory segments...", trajectorySequence.size());

        int trajectoryId = 0;

        schedule(
                new ParallelCommandGroup( // side-quests
                        new HoldExtension(extensionSubsystem, 0),
//                        new HoldArmYawVoltage(extensionSubsystem, ExtensionSubsystem.CENTER_YAW),
                        new SequentialCommandGroup(
                                // TODO: This command currently can break the slide system if the limit switch is not pressed
                                // TODO: Use an absolute encoder so we don't have to do this!
//                        new ResetLift(liftSubsystem),
                                new ParallelCommandGroup(
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++),
                                                PRECISE_POSE_TOLERANCE
                                        ),
                                        new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION)
                                ), // deliver pre-loaded specimen
                                new LogRobotLog(telemetry, "First specimen delivered"),
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(trajectoryId++),
                                        PUSHER_POSE_TOLERANCE
                                ) // first sample pusher
                                        .alongWith( // reset lift
                                                new WaitSeconds(1).andThen(
                                                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // prepare for swing-through
                                                        new SetLift(liftSubsystem, -130, SLIDE_VEL)
                                                )
                                        ),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(trajectoryId++),
                                        PUSHER_POSE_TOLERANCE
                                ), // second sample pusher
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(trajectoryId++),
                                        PUSHER_POSE_TOLERANCE
                                ), // third sample pusher
                                new ParallelCommandGroup(
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++)
                                        ), // prepare for specimen collection
                                        new SequentialCommandGroup(
                                                new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION, SLIDE_VEL), // prepare for intake lift and swing
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN) // prepare for intake claw
                                        )
                                ), // prepare faster
                                new LogRobotLog(telemetry, "Prepare for specimen collection"),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(trajectoryId++),
                                        PRECISE_POSE_TOLERANCE
                                ), // specimen collection
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // grab specimen
                                new WaitSeconds(CLAW_CLOSE_TIME), // wait for claw to close
                                new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION - INTAKE_OFFSET, SLIDE_VEL), // up up and away
                                // GO DELIVER
                                new ParallelCommandGroup(
                                        new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, SLIDE_VEL),
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++)
                                        )
                                ), // submersible delivery system
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN), // deliver specimen
                                new LogRobotLog(telemetry, "Second specimen delivered"),

//                        // ===================== THIRD SPECIMEN =====================

                                new ParallelCommandGroup(
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++)
                                        ), // prepare for specimen collection
                                        new SequentialCommandGroup(
                                                new WaitSeconds(1), // wait for the vehicle to move away from the submersible
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // prepare for swing-through
                                                new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION, SLIDE_VEL), // prepare for intake lift and swing
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN) // prepare for intake claw
                                        ) // hopefully this all completes before the trajectory
                                ), // prepare faster
                                new LogRobotLog(telemetry, "Prepare for specimen collection"),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(trajectoryId++),
                                        PRECISE_POSE_TOLERANCE
                                ), // specimen collection
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // grab specimen
                                new WaitSeconds(CLAW_CLOSE_TIME), // wait for claw to close
                                new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION - INTAKE_OFFSET, SLIDE_VEL), // up up and away
                                new ParallelCommandGroup(
                                        new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, SLIDE_VEL),
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++)
                                        )
                                ), // submersible delivery system
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN), // deliver third specimen
                                new LogRobotLog(telemetry, "Third specimen delivered"),

                                // ===================== FOURTH SPECIMEN =====================

                                new ParallelCommandGroup(
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++)
                                        ), // prepare for specimen collection
                                        new SequentialCommandGroup(
                                                new WaitSeconds(1), // wait for the vehicle to move away from the submersible
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // prepare for swing-through
                                                new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION, SLIDE_VEL), // prepare for intake lift and swing
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN) // prepare for intake claw
                                        ) // hopefully this all completes before the trajectory
                                ), // prepare faster
                                new LogRobotLog(telemetry, "Prepare for specimen collection"),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(trajectoryId++),
                                        PRECISE_POSE_TOLERANCE
                                ), // specimen collection
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // grab specimen
                                new WaitSeconds(CLAW_CLOSE_TIME), // wait for claw to close
                                new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION - INTAKE_OFFSET, SLIDE_VEL), // up up and away
                                new ParallelCommandGroup(
                                        new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, SLIDE_VEL),
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++)
                                        )
                                ), // submersible delivery system
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN),
                                new LogRobotLog(telemetry, "Fourth specimen delivered"),

                                // ===================== FIFTH SPECIMEN =====================

                                new ParallelCommandGroup(
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++)
                                        ), // prepare for specimen collection
                                        new SequentialCommandGroup(
                                                new WaitSeconds(1), // wait for the vehicle to move away from the submersible
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // prepare for swing-through
                                                new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION, SLIDE_VEL), // prepare for intake lift and swing
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN) // prepare for intake claw
                                        ) // hopefully this all completes before the trajectory
                                ), // prepare faster
                                new LogRobotLog(telemetry, "Prepare for specimen collection"),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(trajectoryId++),
                                        PRECISE_POSE_TOLERANCE
                                ), // specimen collection
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // grab specimen
                                new WaitSeconds(CLAW_CLOSE_TIME), // wait for claw to close
                                new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION - INTAKE_OFFSET, SLIDE_VEL), // up up and away
                                new ParallelCommandGroup(
                                        new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, SLIDE_VEL),
                                        new FollowTrajectory(
                                                chassisSubsystem,
                                                AutoConfig.controller,
                                                trajectorySequence.get(trajectoryId++)
                                        )
                                ), // submersible delivery system
                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN), // deliver specimen
                                new LogRobotLog(telemetry, "Fourth specimen delivered"),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(trajectoryId++)
                                ) // go park
                                        .alongWith( // reset lift
                                                new WaitSeconds(1).andThen(
                                                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // prepare for swing-through
                                                        new SetLift(liftSubsystem, -130, SLIDE_VEL)
                                                )
                                        )
                        )
                ));
    }
}
