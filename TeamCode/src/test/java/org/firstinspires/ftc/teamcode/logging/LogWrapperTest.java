package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.util.RobotLog;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;

@RunWith(RobolectricTestRunner.class)
public class LogWrapperTest {
    private static final String LOG_TAG = "LogWrapperTest";

    @Test
    public void testColorConv() {
        RobotLog.dd(LOG_TAG, "DEBUG: %s", LogWrapper.LogLevel.DEBUG.ordinal());
        RobotLog.dd(LOG_TAG, "WARNING: %s", LogWrapper.LogLevel.WARNING.ordinal());
    }
}
