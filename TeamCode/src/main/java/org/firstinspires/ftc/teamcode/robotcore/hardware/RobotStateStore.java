package org.firstinspires.ftc.teamcode.robotcore.hardware;

import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robotcore.game.Alliance;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class RobotStateStore {
    private Pose2d robotPose;
    public static Pose2d staticRobotPose;

    private Alliance alliance;
    public static Alliance staticAlliance;

    public void load() {
        robotPose = staticRobotPose;
        alliance  = staticAlliance;
    }

//    public void save(MyRobot robot) {
////        staticRobotPose = robot.getChassisSubsystem().getLatestPose();
//    }

    public static void reset() {
        staticRobotPose = new Pose2d();
    }
}
