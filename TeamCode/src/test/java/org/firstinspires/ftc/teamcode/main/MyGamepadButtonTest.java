package org.firstinspires.ftc.teamcode.main;

import static org.junit.Assert.assertEquals;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.gamepad.MyGamepadButton;
import org.firstinspires.ftc.teamcode.math.navigation.navigators.AStarNavigator;
import org.firstinspires.ftc.teamcode.math.util.Units;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.maps.CenterStageObstacleMap;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;

@RunWith(RobolectricTestRunner.class)
public class MyGamepadButtonTest {
    @Test
    public void testMyGamepadButton() {
        MyGamepadButton myGamepadButton = new MyGamepadButton();

        for (int i = 0; i < 10; i++) {
            boolean wasJustPressed = myGamepadButton.wasJustPressed(i != 7);
            RobotLog.dd(this.getClass().getSimpleName(), "Just pressed : %s", wasJustPressed);
        }
    }
}
