package org.firstinspires.ftc.teamcode.main;

import static org.junit.Assert.assertEquals;

import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.util.Color;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;
import org.robolectric.annotation.Config;
import org.robolectric.shadows.ShadowLog;

import java.util.Arrays;

@RunWith(RobolectricTestRunner.class)
public class ColorConvTest {
    @Test
    public void testColorConv() {
        double[] outputHSV = Color.convertRGBtoHSV(255, 255, 255);

        RobotLog.dd(this.getClass().getSimpleName(), "color : %s %s %s", outputHSV[0], outputHSV[1], outputHSV[2]);

        assertEquals(new double[] { 0, 0, 1 }, Arrays.stream(outputHSV).toArray());
    }
}
