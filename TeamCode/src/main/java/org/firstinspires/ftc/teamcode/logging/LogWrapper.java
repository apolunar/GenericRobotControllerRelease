package org.firstinspires.ftc.teamcode.logging;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.RobotLog;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collection;

public class LogWrapper {
    public static LogLevel logLevel = LogLevel.WARNING;

    public static Telemetry telemetry;

    private static Telemetry dashboardTelemetry;
    private static FtcDashboard dashboard;
    private static TelemetryPacket packet = new TelemetryPacket();

    public enum LogTarget {
        TELEMETRY,
        DASHBOARD,
        LOGCAT
    }

    public enum LogLevel {
        VERBOSE,
        DEBUG,
        WARNING,
        ERROR
    }

    /**
     * TODO: Add more features from <a href="https://acmerobotics.github.io/ftc-dashboard/features#telemetry">FTC Dashboard</a>
     */
    private LogWrapper() {}

    /**
     * Call me as early as possible
     * @param telemetry
     */
    public static void init(Telemetry telemetry, LogLevel logLevel) {
        LogWrapper.logLevel = logLevel;
        LogWrapper.telemetry = telemetry;

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    public static void init(Telemetry telemetry) {
        telemetry = telemetry;
    }

    /**
     * TODO: Add overflow methods for default behavior
     *  - Also consider removing String.format, and using `message` and then args
     * @param logTargets
     * @param tag
     * @param format
     * @param args
     */
    public static void log(Collection<LogTarget> logTargets, LogLevel logLevel,
                           String tag, String format, Object... args) {
        if (logTargets.contains(LogTarget.TELEMETRY) && telemetry != null) {
            if (logLevel.ordinal() < LogWrapper.logLevel.ordinal()) return;
            telemetry.addData(tag, String.format(format, args));
        }
        if (logTargets.contains(LogTarget.DASHBOARD) && dashboardTelemetry != null) {
            if (logLevel.ordinal() < LogWrapper.logLevel.ordinal()) return;
            packet.put(tag, String.format(format, args));
            dashboard.sendTelemetryPacket(packet);
            packet = new TelemetryPacket();
        }
        if (logTargets.contains(LogTarget.LOGCAT)) {
            switch (logLevel) {
                case VERBOSE:
                    RobotLog.vv(tag, format, args);
                    break;
                case DEBUG:
                    RobotLog.dd(tag, format, args);
                    break;
                case WARNING:
                    RobotLog.ww(tag, format, args);
                    break;
                case ERROR:
                    RobotLog.ee(tag, format, args);
                    break;
            }
        }
    }

    public static void log(LogTarget logTarget, LogLevel logLevel, String tag, String format, Object... args) {
        log(List.of(logTarget), logLevel, tag, format, args );
    }

    public static void log(LogTarget[] logTarget, String tag, String format, Object... args) {
        log(Arrays.asList(logTarget), logLevel, tag, format, args);
    }

    public static void log(LogTarget logTarget, String tag, String format, Object... args) {
        log(logTarget, logLevel, tag, format, args);
    }

    public static void log(String tag, String format, Object... args) {
        log(List.of(LogTarget.LOGCAT, LogTarget.DASHBOARD), logLevel, tag, format, args);
    }
}
