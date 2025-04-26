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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySegment;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.command.lift.SetLiftClaw;
import org.firstinspires.ftc.teamcode.robotcore.command.lift.SetLift;
import org.firstinspires.ftc.teamcode.robotcore.command.lift.ResetLift;
import org.firstinspires.ftc.teamcode.robotcore.command.action.WaitSeconds;
import org.firstinspires.ftc.teamcode.robotcore.command.chassis.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

@Autonomous
@Disabled
public class Specimen extends CommandOpMode {
    public static int SLIDE_VEL = 1400;
    public static int SLOW_VEL = 20;
    public static int SLOW_ACCEL = 40;

    /**
     * Utility to generate sample Y
     * TODO: Seems a bit unnecessary but whatever
     *
     * @param sample
     * @return the Y position of the sample
     */
    private static double sampleY(int sample) {
        return ObstacleMap.INCHES_PER_TILE + 6.7 * (sample + 1);
    }

    private static TrajectorySegment samplePusher(int sample) {
        double lineupY = sampleY(sample) - 14 + sample * 2.2;
        return new TrajectorySegment( // to the sample
                Rotation2d.fromDegrees(-180),
                new Translation2d[]{ // the funky-looking math is to offset it with sample 1 and 2
                        new Translation2d(ObstacleMap.INCHES_PER_TILE, lineupY),
                        new Translation2d(ObstacleMap.INCHES_PER_TILE * 2, lineupY),
                        new Translation2d(ObstacleMap.INCHES_PER_TILE * 2, sampleY(sample))
                },
                new Pose2d(10, sampleY(sample), Rotation2d.fromDegrees(-180)),
                Rotation2d.fromDegrees(0),
                AutoConfig.TRAJECTORY_CONFIG
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
                new Pose2d(-2 - specimen / 2.2, sampleY(0), Rotation2d.fromDegrees(-180)),
                Rotation2d.fromDegrees(0),
                new TrajectoryConfig(SLOW_VEL, SLOW_ACCEL)
        );
    }

    private static TrajectorySegment backToSpecimenCollection() {
        return new TrajectorySegment(
                Rotation2d.fromDegrees(-180),
                new Translation2d[]{
                        new Translation2d(ObstacleMap.INCHES_PER_TILE, 0)
                },
                new Pose2d(ObstacleMap.INCHES_PER_TILE - 5, sampleY(0), Rotation2d.fromDegrees(-180)),
                Rotation2d.fromDegrees(0),
                AutoConfig.TRAJECTORY_CONFIG
        );
    }

    private static TrajectorySegment submersibleDeliverySystem(int specimen) {
        return new TrajectorySegment( // drive forward
                Rotation2d.fromDegrees(0),
                new Translation2d[]{
                        new Translation2d(ObstacleMap.INCHES_PER_TILE, -10)
                },
                // NOTE: Do not change the `+ 5` IT WILL CAUSE A COMPLETELY UNEXPLAINED RUNTIME ERROR
                // something fails with the trajectory generation
                new Pose2d(ObstacleMap.INCHES_PER_TILE + 11 + specimen/3., -14, Rotation2d.fromDegrees(0)),
                Rotation2d.fromDegrees(0),
                AutoConfig.TRAJECTORY_CONFIG
        );
    }

    private static TrajectorySegment slideToTheLeft() {
        return new TrajectorySegment(
                Rotation2d.fromDegrees(-90),
                new Translation2d[0],
                new Pose2d(ObstacleMap.INCHES_PER_TILE + 11.5, -11, Rotation2d.fromDegrees(-90)),
                Rotation2d.fromDegrees(0),
                AutoConfig.TRAJECTORY_CONFIG
        );
        // UP NEXT: CRISS CROSS
    }

    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);

        // TODO: Add failsafe to log ERROR for generation failure
        // should really be a compile-time error
        // TODO: How many inches is this trajectory
        TrajectorySegment[] trajectorySegments = new TrajectorySegment[]{
                submersibleDeliverySystem(0),
                slideToTheLeft(),
                samplePusher(0), // 0 is outermost sample
                samplePusher(1),
//                samplePusher(2),
                // holy baloney
                // specimen collection three times
                // first go forward to prepare for specimen collection
                new TrajectorySegment(
                        Rotation2d.fromDegrees(0),
                        new Translation2d[0],
                        new Pose2d(ObstacleMap.INCHES_PER_TILE + 5, sampleY(0), Rotation2d.fromDegrees(0)),
                        Rotation2d.fromDegrees(0),
                        AutoConfig.TRAJECTORY_CONFIG
                ),
                specimenCollection(0),
                submersibleDeliverySystem(1),
                slideToTheLeft(),
                backToSpecimenCollection(), // payload one delivered
                // TODO: Add failsafe for if robot gets stuck or runs into something or times out
                specimenCollection(1),
                submersibleDeliverySystem(2),
                slideToTheLeft(),
                new TrajectorySegment(
                        Rotation2d.fromDegrees(-180),
                        new Translation2d[0],
                        new Pose2d(5, sampleY(0), Rotation2d.fromDegrees(-180)),
                        Rotation2d.fromDegrees(0),
                        AutoConfig.TRAJECTORY_CONFIG
                ) // park
                // payload three removed :sob:
//                specimenCollection(),
//                submersibleDeliverySystem(),
//                slideToTheLeft() // payload three delivered
        };

        // TODO: Functions should be more like macros so that the same traj doesn't keep needing to be regenerated but whatever...
        TrajectorySequence trajectorySequence = TrajectorySequence.weaveTrajectorySequence(trajectorySegments);

        RobotLog.dd(this.getClass().getSimpleName(), "Preparing to execute %s trajectory segments...", trajectorySequence.size());

        // TODO: Possibly simplify schedule with method
        // ...would prefer this to be a lambda so I can access the trajectorySequence!
//        CommandBase[] preloadedSpecimenDelivery = generateSpecimenDelivery();

        // TODO: WHAT A MESS!
        // TODO: Also, don't try to solve problems you don't have
        //  as while it is annoying that you have to type each subsystem it doesn't really effect functionality
        // TODO: Add flowchart-like features such that each command can optionally have a failure condition and failure mode
        schedule(
                new SequentialCommandGroup(
                        // TODO: This command currently can break the slide system if the limit switch is not pressed
                        // TODO: Use an absolute encoder so we don't have to do this!
                        new ResetLift(liftSubsystem),
                        new ParallelCommandGroup(
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(0)
                                ),
                                new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, SLIDE_VEL)
                        ), // deliver pre-loaded specimen
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(1)
                        ), // slide to the left
                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN), // release
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(2)
                        ) // first sample pusher
                                .alongWith( // reset lift
                                        new WaitSeconds(1).andThen(
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // prepare for swing-through
                                                new SetLift(liftSubsystem, -50, SLIDE_VEL)
                                        )
                                ),
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(3)
                        ), // second sample pusher
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(4)
                        ), // prepare for specimen collection
                        new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION, SLIDE_VEL), // prepare for intake lift and swing
                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN), // prepare for intake claw
//                        new MoveSwing(liftSubsystem, false), // NOTE: Should already have moved via MoveLift,
//                         otherwise the below command will prevent it from moving because the commands complete instantly
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(5)
                        ), // specimen collection
                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // grab specimen
                        new WaitSeconds(0.5), // wait for claw to close
                        new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION - 100, SLIDE_VEL), // up up and away
                        // GO DELIVER
                        new ParallelCommandGroup(
                                new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, SLIDE_VEL),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(6)
                                )
                        ), // submersible delivery system
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(7)
                        ), // slide to the left
                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN), // deliver specimen
                        // ===================== THIRD SPECIMEN =====================
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(8)
                        ) // prepare for second specimen collection
                                .alongWith( // reset lift
                                        new WaitSeconds(1).andThen(
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // prepare for swing-through
                                                new SetLift(liftSubsystem, -50, SLIDE_VEL)
                                        )
                                ),
                        new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION, SLIDE_VEL), // also swings
                        new WaitSeconds(0.5),
                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN), // get ready to grab second specimen
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(9)
                        ), // second specimen collection
                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // grab specimen
                        new WaitSeconds(0.5), // wait for claw to close
                        new SetLift(liftSubsystem, LiftSubsystem.INTAKE_POSITION - 100, SLIDE_VEL), // up up and away
                        new ParallelCommandGroup(
                                new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, SLIDE_VEL),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        trajectorySequence.get(10)
                                )
                        ), // second submersible delivery system
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(11)
                        ), // second slide to the left
                        new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.OPEN), // deliver second specimen
                        new FollowTrajectory(
                                chassisSubsystem,
                                AutoConfig.controller,
                                trajectorySequence.get(12)
                        ) // go park
                                .alongWith( // reset lift
                                        new WaitSeconds(1).andThen(
                                                new SetLiftClaw(liftSubsystem, LiftSubsystem.ClawPosition.CLOSED), // prepare for swing-through
                                                new SetLift(liftSubsystem, -50, SLIDE_VEL)
                                        )
                                )
                )
        );
    }
}
