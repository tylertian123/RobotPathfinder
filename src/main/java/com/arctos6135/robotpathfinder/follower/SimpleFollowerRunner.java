package com.arctos6135.robotpathfinder.follower;

/**
 * A simple follower runner that uses {@link Thread#sleep(long)} to time the
 * control loop.
 * 
 * <p>
 * Note that since {@link Thread#sleep(long)} is used to delay between control
 * loop executions, the frequency will not be exact as it does not take into
 * account the execution time of the control loop. For precise timing, use a
 * {@link TimedFollowerRunner}.
 * </p>
 * 
 * @author Tyler Tian
 * @see FollowerRunner
 * @see TimedFollowerRunner
 * @since 3.0.0
 */
public class SimpleFollowerRunner implements FollowerRunner {

    private volatile Follower<?> follower;
    private Runner runner;
    private long delay;

    /**
     * Creates a new follower runner.
     */
    public SimpleFollowerRunner() {
    }

    @Override
    public void start(Follower<?> follower, int frequency) {
        if (runner != null) {
            return;
        }
        this.follower = follower;
        delay = Math.round(1000.0 / frequency);
        runner = new Runner();
        runner.start();
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
        if (runner.isAlive()) {
            // This calls stop
            runner.interrupt();
            try {
                // Make sure that the thread actually ended
                runner.join();
            } catch (InterruptedException e) {
                throw new IllegalStateException("Thread.join() was interrupted!", e);
            } finally {
                runner = null;
                follower = null;
                delay = 0;
            }
        }
    }

    /**
     * The thread that runs the control loop. This is a daemon thread.
     */
    private class Runner extends Thread {

        public Runner() {
            setDaemon(true);
        }

        @Override
        public void run() {
            while (!Thread.interrupted()) {
                // Run and check if finished
                follower.run();
                if (follower.isFinished()) {
                    break;
                }

                // Delay
                try {
                    Thread.sleep(delay);
                } catch (InterruptedException e) {
                    break;
                }
            }
            // Stop the follower if interrupted
            follower.stop();
        }
    }
}
