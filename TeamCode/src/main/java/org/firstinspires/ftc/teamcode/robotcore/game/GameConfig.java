package org.firstinspires.ftc.teamcode.robotcore.game;

import com.acmerobotics.dashboard.config.Config;

import lombok.Builder;
import lombok.Getter;

@Getter
@Builder
@Config
public class GameConfig {
    @Builder.Default
    private Alliance alliance = Alliance.NONE;
}
