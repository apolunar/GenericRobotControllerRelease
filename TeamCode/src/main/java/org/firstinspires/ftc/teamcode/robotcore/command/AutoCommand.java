package org.firstinspires.ftc.teamcode.robotcore.command;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.teamcode.math.controller.DriveController;
import org.firstinspires.ftc.teamcode.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.math.navigation.Navigator;
import org.firstinspires.ftc.teamcode.robotcore.game.Alliance;
import org.firstinspires.ftc.teamcode.robotcore.game.GameScorer;
import org.firstinspires.ftc.teamcode.robotcore.game.GameStage;
//import org.firstinspires.ftc.teamcode.robotcore.command.scorer.DroneScorer;
//import org.firstinspires.ftc.teamcode.robotcore.hardware.MyRobot;
//import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotState;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotStateStore;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
//import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.DroneSubsystem;
//import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ScannerSubsystem;
//import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ScrewSubsystem;
//import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.SlideSubsystem;
//import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.SpatulaSubsystem;

import java.util.function.DoubleSupplier;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AutoCommand extends CommandBase {
    private final RobotStateStore robotStateStore;

    private final ChassisSubsystem chassisSubsystem;
    private final CameraSubsystem cameraSubsystem;


    private final DoubleSupplier runtimeSupplier;

    private GameScorer[] gameScorers;
    private GameScorer activeGameScorer;

    private HolonomicDriveController controller;
    private Navigator navigator;
    private TrajectoryConfig trajectoryConfig;

    @Override
    public void initialize() {
//        controller = MyRobot.RobotConfig.controller;
//        navigator  = MyRobot.RobotConfig.navigator;

//        trajectoryConfig    = MyRobot.RobotConfig.trajectoryConfig;

        gameScorers = new GameScorer[] {

        };
        activeGameScorer = nextGameScorer();

        // this will not work because init is on first sched and also active game scorer is never sched again after first complete
        CommandScheduler.getInstance().schedule((Command) activeGameScorer);
        CommandScheduler.getInstance().onCommandFinish((good) -> {
            if (!activeGameScorer.isRepeatable()) {
                activeGameScorer.setScorerExhausted(true);
            }
            activeGameScorer = nextGameScorer();
        });
    }

    @Override
    public void execute() {
        // TODO:
    }

    private GameScorer nextGameScorer() {
//        return getNextGameScorer(autoConfig.getMaxDepth(), 0,  runtimeSupplier.getAsDouble(), chassisSubsystem.getLatestPose(), GameStage.ENDGAME.getGameStage(runtimeSupplier.getAsDouble()));
        return null;
    }

    private GameScorer getNextGameScorer(int maxDepth, int currentDepth, double runtime, Pose2d robotPose, GameStage currentGameStage) {
        GameScorer bestGameScorer = null;
        double bestPointTimeRatio = Integer.MAX_VALUE;
        for (GameScorer gameScorer : gameScorers) {
            if (gameScorer.getGameStage() != currentGameStage || gameScorer.isScorerExhausted()) continue;

            double estimatedEndTime = runtime + gameScorer.getEstimatedActionTime(); // TODO: Add estimated movement time
            if (estimatedEndTime > currentGameStage.cutoff) continue;

            double pointTimeRatio = estimatedEndTime / gameScorer.getScorePoints();

            int nextDepth = currentDepth + 1;

            GameScorer nextBestGameScorer;
            Pose2d estimatedEndPose = gameScorer.getEstimatedEndPose();
            if (nextDepth < maxDepth) {
                nextBestGameScorer = getNextGameScorer(
                        maxDepth,
                        nextDepth,
                        estimatedEndTime,
                        estimatedEndPose,
                        GameStage.OPENING.getGameStage(estimatedEndTime)
                );
                pointTimeRatio += estimatedEndTime / nextBestGameScorer.getScorePoints();
            }

            if (pointTimeRatio < bestPointTimeRatio) {
                bestPointTimeRatio = pointTimeRatio;
                bestGameScorer = gameScorer;
            }
        }

        return bestGameScorer;
    }
}