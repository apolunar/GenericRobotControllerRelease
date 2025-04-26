package org.firstinspires.ftc.teamcode.main;

import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.util.Range;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;

@RunWith(RobolectricTestRunner.class)
public class RangeTest {
    @Test
    public void testNextGameScorer() {
        boolean valueWithinPercentage = Range.valueWithinPercentage(-1570, -1875, .25);
        RobotLog.dd(this.getClass().getSimpleName(), "%s", valueWithinPercentage);
        assertTrue(valueWithinPercentage);
    }
}
