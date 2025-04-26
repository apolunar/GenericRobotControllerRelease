package org.firstinspires.ftc.teamcode.math.trajectory;

import android.util.Pair;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * TODO: Replace static methods with class methods
 * WARNING: This will break a lot of things!
 */
public class TrajectorySequence extends ArrayList<Pair<Trajectory, Rotation2d>> {
    public static TrajectorySequence weaveTrajectorySequence(Translation2d startPosition, TrajectorySegment[] trajectorySegments) {
        TrajectorySequence trajectories = new TrajectorySequence();

        for (int i = 0; i < trajectorySegments.length; i++) {
            TrajectorySegment segment = trajectorySegments[i];

            trajectories.add(
                    new Pair<>(
                            TrajectoryGenerator.generateTrajectory(
                                    i == 0 ? new Pose2d(startPosition, segment.start) : new Pose2d(trajectorySegments[i - 1].end.getTranslation(), segment.start), // use the end of the previous trajectory as the start for this one
                                    Arrays.asList(segment.waypoints),
                                    segment.end,
                                    segment.trajectoryConfig
                            ),
                            segment.desiredFinalHeading
                    )
            );
        }

        return trajectories;
    }

    public static TrajectorySequence weaveTrajectorySequence(TrajectorySegment[] trajectorySegments) {
        return weaveTrajectorySequence(new Translation2d(0, 0), trajectorySegments);
    }

    public static void logTrajectorySequence(TrajectorySequence trajectorySequence) {
        for (Pair<Trajectory, Rotation2d> trajectory : trajectorySequence) {
            for (Trajectory.State state : trajectory.first.getStates()) {
                RobotLog.dd("TRAJECTORY", "State : %s", state);
            }
            RobotLog.dd("TRAJECTORY", "Target rotation : %s deg", trajectory.second.getDegrees());
            RobotLog.dd("TRAJECTORY", "Runtime         : %s s", trajectory.first.getTotalTimeSeconds());
        }

        double totalRuntime = 0;
        for (Pair<Trajectory, Rotation2d> trajectory : trajectorySequence) {
            RobotLog.dd("TRAJECTORY", "Summary         : %s", trajectory.first.getInitialPose());
            RobotLog.dd("TRAJECTORY", "Target rotation : %s deg", trajectory.second.getDegrees());
            RobotLog.dd("TRAJECTORY", "Runtime         : %s s", trajectory.first.getTotalTimeSeconds());
            totalRuntime += trajectory.first.getTotalTimeSeconds();
        }

        RobotLog.dd("TRAJECTORY", "Total runtime : %s s", totalRuntime);
    }

    public static void logTrajectory(Trajectory trajectory) {
        for (Trajectory.State state : trajectory.getStates()) {
            RobotLog.dd("TRAJECTORY", "State : %s", state);
        }
    }
}
