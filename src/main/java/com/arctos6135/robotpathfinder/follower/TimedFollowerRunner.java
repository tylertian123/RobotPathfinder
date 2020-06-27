package com.arctos6135.robotpathfinder.follower;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * A follower runner that uses a {@link ScheduledExecutorService} to time the
 * control loop.
 * 
 * <p>
 * Because this implementation uses
 * {@link ScheduledExecutorService#scheduleAtFixedRate(Runnable, long, long, TimeUnit)},
 * which takes into account the execution time, the control loop will be run at
 * a precise rate. If precise timing is not needed, a
 * {@link SimpleFollowerRunner} can be used instead.
 * </p>
 * 
 * @author Tyler Tian
 * @see FollowerRunner
 * @see SimpleFollowerRunner
 * @since 3.0.0
 */
public class TimedFollowerRunner implements FollowerRunner {

    private volatile Follower<?> follower;
    private ScheduledExecutorService executor;
    private ScheduledFuture<?> future;

    /**
     * Creates a new follower runner.
     */
    public TimedFollowerRunner() {
    }

    @Override
    public void start(Follower<?> follower, int frequency) {
        if (future != null) {
            return;
        }
        this.follower = follower;

        long delayMicro = Math.round(1000000.0 / frequency);
        // Use a ScheduledExecutor instead of a Timer
        executor = Executors.newSingleThreadScheduledExecutor();
        future = executor.scheduleAtFixedRate(() -> follower.run(), 0, delayMicro, TimeUnit.MICROSECONDS);
    }

    @Override
    public boolean isFinished() {
        if (follower == null) {
            return true;
        }
        return follower.isFinished();
    }

    /**
     * {@inheritDoc}
     * 
     * <p>
     * Note that this method will block until the control loop thread has been
     * stopped.
     * </p>
     * 
     * @throws IllegalStateException If interrupted while waiting for the control
     *                               loop thread to finish
     */
    @Override
    public void stop() {
        if (future == null) {
            return;
        }
        // Shut down the ScheduledExecutorService completely
        future.cancel(true);
        executor.shutdownNow();
        try {
            executor.awaitTermination(Long.MAX_VALUE, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            throw new IllegalStateException("ExecutorService.awaitTermination() was interrupted!", e);
        } finally {
            follower.stop();

            follower = null;
            future = null;
            executor = null;
        }
    }
}
