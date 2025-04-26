package org.firstinspires.ftc.teamcode.robotcore.game;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import java.util.ArrayList;

import lombok.Getter;

/**
 * Just automatically creates a flipped/unflipped trajectory based on alliance
 */
@Getter
public class AllianceTrajectory {
    /**
     * See <a href="https://firstinspiresst01.blob.core.windows.net/first-in-show-ftc/game-manual-part-2-traditional.pdf">gm2</a> and the official <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html">FCS</a> reference
     * @return a trajectory adjusted for alliance
     */
    public static Trajectory generateTrajectory(ArrayList<Pose2d> waypoints, TrajectoryConfig trajectoryConfig, Alliance alliance) {
        if (alliance != Alliance.RED_ALLIANCE) {
            for (int waypoint = 0; waypoint < waypoints.size(); waypoint++) {
                Pose2d thisWaypoint = waypoints.get(waypoint);

                // Flip waypoint
                waypoints.set(waypoint, new Pose2d(-thisWaypoint.getX(), thisWaypoint.getY(), thisWaypoint.getRotation()));
            }
        }

        return TrajectoryGenerator.generateTrajectory(waypoints, trajectoryConfig);
    }
}
