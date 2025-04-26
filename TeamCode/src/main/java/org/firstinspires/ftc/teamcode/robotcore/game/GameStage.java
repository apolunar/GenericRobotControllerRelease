package org.firstinspires.ftc.teamcode.robotcore.game;

public enum GameStage {
    OPENING(30),
    MIDGAME(120),
    ENDGAME(150),
    NONE(Integer.MAX_VALUE);
    public final int cutoff;

    GameStage(int cutoff) {
        this.cutoff = cutoff;
    }

    public GameStage getGameStage(double runtime) {
        if (runtime < GameStage.OPENING.cutoff) return GameStage.OPENING;
        else if (runtime < GameStage.MIDGAME.cutoff) return GameStage.MIDGAME;
        else if (runtime < GameStage.ENDGAME.cutoff) return GameStage.ENDGAME;
        else return GameStage.NONE;
    }
}