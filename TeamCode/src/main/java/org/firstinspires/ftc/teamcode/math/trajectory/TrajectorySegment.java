package org.firstinspires.ftc.teamcode.math.trajectory;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;

import java.util.ArrayList;

import lombok.RequiredArgsConstructor;

/**
 * A utility class that allows users to easily generate a sequence of trajectories that fits into our framework
 */
@RequiredArgsConstructor
public class TrajectorySegment {
    public final Rotation2d start; // is assumed to only care about the direction in which you start driving
    public final Translation2d[] waypoints;
    public final Pose2d end;
    public final Rotation2d desiredFinalHeading;
    public final TrajectoryConfig trajectoryConfig;

    public static Pose2d getEndState(Trajectory trajectory) {
        return trajectory.getStates().get(trajectory.getStates().size()-1).poseMeters;
    }
}
