package org.firstinspires.ftc.teamcode.main;

import android.util.Log;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.filters.SmallTest;
import static org.junit.Assert.*;

import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;

@RunWith(AndroidJUnit4.class)
@SmallTest
public class HelloOnAndroidTest {
    private static final String LOG_TAG = "TaskHelloOnAndroid";

    @Before
    public void setUp() throws Exception {
        // TODO: Figure out how to run opmode like FtcDashboard
    }

    @Test
    public void testHelloAndroid() {
        Log.i(LOG_TAG, "Test: Start");
        assertEquals( "Hello", 1, 1);
    }
}