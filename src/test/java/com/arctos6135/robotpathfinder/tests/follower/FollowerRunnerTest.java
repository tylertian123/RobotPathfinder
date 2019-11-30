package com.arctos6135.robotpathfinder.tests.follower;

import static org.hamcrest.Matchers.both;
import static org.hamcrest.Matchers.greaterThanOrEqualTo;
import static org.hamcrest.Matchers.is;
import static org.hamcrest.Matchers.lessThanOrEqualTo;
import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.follower.FollowerRunner;
import com.arctos6135.robotpathfinder.follower.SimpleFollowerRunner;
import com.arctos6135.robotpathfinder.follower.TimedFollowerRunner;

import org.junit.Test;

public class FollowerRunnerTest {

    private volatile boolean runCalled = false;
    private volatile boolean stopCalled = false;
    private volatile boolean initializeCalled = false;

    /**
     * Tests a {@link FollowerRunner}.
     * 
     * This will perform tests on all the methods of the {@link FollowerRunner},
     * making sure that the follower's methods are being called correctly.
     * 
     * @param runner The {@link FollowerRunner} to test
     */
    public void testFollowerRunner(FollowerRunner runner) {
        Follower<?> f = new Follower<Moment>() {

            @Override
            protected void _initialize() {
                initializeCalled = true;
            }

            @Override
            protected boolean _run() {
                runCalled = true;
                return false;
            }

            @Override
            protected void _stop() {
                stopCalled = true;
            }
        };

        runner.start(f, 50);
        // Test initialize
        try {
            // Allocate time for thread startup
            Thread.sleep(20);
        } catch (InterruptedException e) {
            runner.stop();
            throw new RuntimeException("Interrupted!", e);
        }
        assertThat("initialize() should be called", initializeCalled, is(true));

        // Test run
        runCalled = false;
        while (!runCalled) {
            try {
                // Sleep half a millisecond at a time
                Thread.sleep(0, 500000);
            } catch (InterruptedException e) {
                runner.stop();
                throw new RuntimeException("Interrupted!", e);
            }
        }
        long start = System.nanoTime();
        runCalled = false;
        while (!runCalled) {
            if (System.nanoTime() - start > 100000000L) {
                runner.stop();
                fail("Follower.run() was not called within 100ms!");
            }
            try {
                // Sleep half a millisecond at a time
                Thread.sleep(0, 500000);
            } catch (InterruptedException e) {
                runner.stop();
                throw new RuntimeException("Interrupted!", e);
            }
        }
        assertThat("run() should be called within the correct time period",
                (int) ((System.nanoTime() - start) / 1000000),
                both(greaterThanOrEqualTo(19)).and(lessThanOrEqualTo(21)));

        // Test stop
        stopCalled = false;
        runner.stop();
        runCalled = false;

        try {
            Thread.sleep(40);
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted!", e);
        }

        assertThat("stop() should be called after the runner is stopped", stopCalled, is(true));
        assertThat("run() should no longer be called after the follower is stopped", runCalled, is(false));
    }

    /**
     * Tests {@link SimpleFollowerRunner}.
     * 
     * This tests all the methods of {@link SimpleFollowerRunner}.
     */
    @Test
    public void testSimpleFollowerRunner() {
        testFollowerRunner(new SimpleFollowerRunner());
    }

    /**
     * Tests {@link TimedFollowerRunner}.
     * 
     * This tests all the methods of {@link TimedFollowerRunner}.
     */
    @Test
    public void testTimedFollowerRunner() {
        testFollowerRunner(new TimedFollowerRunner());
    }
}
