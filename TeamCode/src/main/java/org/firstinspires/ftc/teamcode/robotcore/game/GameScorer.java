package org.firstinspires.ftc.teamcode.robotcore.game;

import com.arcrobotics.ftclib.geometry.Pose2d;

/**
 * How we interface with the game
 * IMPORTANT: All Pose2D are in global/field coordinates
 */
public interface GameScorer {
    GameStage getGameStage();

    int getScorePoints();

    /**
     * The number of seconds the ACTION (not chassis navigation) should take
     */
    double getEstimatedActionTime();

    /**
     * Allows calculation of next action completion time
     */
    Pose2d getEstimatedEndPose();

    /**
     * Can the action be completed more than once for score?
     */
    boolean isRepeatable();

    /**
     * Don't run this scorer again
     * Author's note: this is a setter because it could happen that
     * some event un-exhausts this scorer, so better to just prepare for that event
     * Furthermore, even on repeatable events you could run out of game elements,
     * better to leave that figuring to AutoCommand
     */
    void setScorerExhausted(boolean value);

    boolean isScorerExhausted();
}
