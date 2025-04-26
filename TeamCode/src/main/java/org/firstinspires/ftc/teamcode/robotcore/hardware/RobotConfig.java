package org.firstinspires.ftc.teamcode.robotcore.hardware;

import com.acmerobotics.dashboard.config.Config;

import lombok.Builder;
import lombok.Getter;

/**
 * TODO: To release this it's ideal not to be referring to statically defined constants
 *  when using commonly used classes like TrajectorySequence and FollowTrajectory but for now it works.
 */
@Getter
@Builder
@Config
public class RobotConfig {
    public static final boolean DEBUG = true;

    @Builder.Default
    private RobotStateStore robotStateStore = RobotStateStore.builder().build();
    @Builder.Default
    private boolean saveRobotState = false;
}
