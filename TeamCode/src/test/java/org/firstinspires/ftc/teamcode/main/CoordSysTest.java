package org.firstinspires.ftc.teamcode.main;

import static org.junit.Assert.assertEquals;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.navigation.CoordSys;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.StartingPose;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;
import org.robolectric.shadows.ShadowLog;

import java.util.ArrayList;

@RunWith(RobolectricTestRunner.class)
public class CoordSysTest {
    @Test
    public void coordSysConvTest() {
        Pose2d resultRedBackStage = CoordSys.fieldToRobot(
                StartingPose.RED_BACKSTAGE.pose
        );
        Pose2d resultRedAudience = CoordSys.fieldToRobot(
                StartingPose.RED_AUDIENCE.pose
        );
        Pose2d resultBlueBackStage = CoordSys.fieldToRobot(
                StartingPose.BLUE_BACKSTAGE.pose
        );
        Pose2d resultBlueAudience = CoordSys.fieldToRobot(
                StartingPose.BLUE_AUDIENCE.pose
        );

        RobotLog.dd("COORDSYS", "Results : (red backstage: %s red audience: %s blue backstage: %s blue audience: %s)", resultRedBackStage, resultRedAudience, resultBlueBackStage, resultBlueAudience);
    }
}
