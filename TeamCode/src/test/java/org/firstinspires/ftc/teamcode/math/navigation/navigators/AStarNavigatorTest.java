package org.firstinspires.ftc.teamcode.math.navigation.navigators;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.math.navigation.Navigator;
import org.firstinspires.ftc.teamcode.math.util.Units;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.maps.CenterStageObstacleMap;
import static org.junit.Assert.*;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

@RunWith(RobolectricTestRunner.class)
@Config(manifest=Config.NONE)
public class AStarNavigatorTest {
//    @Test
    public void testTrajectoryAngle() {
        Translation2d origin1 = new Translation2d(-4, -4);
        Translation2d target1 = new Translation2d(-3, -3);
        
        double trajectoryAngle1 = Navigator.angleBetween(origin1, target1);

        assertEquals(Math.toRadians(45), trajectoryAngle1, 0.1);

        Translation2d origin2 = new Translation2d(-66, -66);
        Translation2d target2 = new Translation2d(-66, -65.92);

        double trajectoryAngle2 = Navigator.angleBetween(origin2, target2);

        assertEquals(Math.toRadians(45), trajectoryAngle2, 0.1);
    }
    
    @Test
    public void testIntegratedAStarNavigation() {
        CenterStageObstacleMap obstacleMap = new CenterStageObstacleMap();

//        AStarNavigator navigator = new AStarNavigator(
//                obstacleMap,
//                Units.feetToInches(12),
//                MyRobot.RobotConfig.ROBOT_SIZE,
//                MyRobot.RobotConfig.trajectoryConfig
//        );
//
//        Translation2d source = new Translation2d(Units.feetToInches(-4.7), Units.feetToInches(-4.7));
//        Translation2d target = new Translation2d(Units.feetToInches(3), Units.feetToInches(3.8));
//
//        // TODO: if y > ~1ft (center barrier distance) then add obstacle for barrier because you cannot go back through it and remove once on the other side
//        Trajectory trajectory = navigator.search(
//                source,
//                target
//        );

//        assertEquals(target.getTranslation(), trajectory.getStates().get(trajectory.getStates().size()-1).poseMeters.getTranslation());
    }
}
