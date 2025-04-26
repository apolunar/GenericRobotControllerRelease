package org.firstinspires.ftc.teamcode.main;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import org.firstinspires.ftc.teamcode.math.navigation.CoordSys;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.math.util.Units;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;

import java.util.Arrays;

@RunWith(RobolectricTestRunner.class)
public class ClampedCubicGenerationTest {
    @Test
    public void testClampedCubic() {
//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//                CoordSys.fieldToRobot(new Pose2d(Units.feetToInches(-5.5), Units.feetToInches(-5.5), Rotation2d.fromDegrees(0))),
//                Arrays.asList(
//                        CoordSys.fieldToRobot(new Translation2d(
//                                Units.feetToInches(-3),
//                                Units.feetToInches(-3))
//                        ).getTranslation(),
//                        CoordSys.fieldToRobot(new Translation2d(
//                                Units.feetToInches(-3.6),
//                                Units.feetToInches(-3.6)
//                        )).getTranslation(),
//                        CoordSys.fieldToRobot(new Translation2d(
//                                Units.feetToInches(0),
//                                Units.feetToInches(0)
//                        )).getTranslation()
//                ),
//                CoordSys.fieldToRobot(new Pose2d(Units.feetToInches(5.5), Units.feetToInches(5.5), Rotation2d.fromDegrees(0))),
//                MyRobot.RobotConfig.trajectoryConfig
//        );

//        TrajectorySequence.logTrajectory(trajectory);
    }
}
