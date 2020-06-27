package com.arctos6135.robotpathfinder.follower;

/**
 * Follower runners are convenient tools for running {@link Follower} control
 * loops.
 * 
 * <p>
 * Follower runners allow you to easily run a {@link Follower}'s control loop at
 * a specific frequency concurrently. Unlike a {@link Follower}, which requires
 * calling {@link Follower#run()} repeatedly, follower runners will keep running
 * once {@link #start(Follower, int)} has been called, and will not stop until
 * the following is finished or {@link #stop()} has been called.
 * </p>
 * <p>
 * Note that follower runners can be reused, but can only run one follower at a
 * time.
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public interface FollowerRunner {

    /**
     * Starts running a follower at a desired frequency.
     * 
     * <p>
     * If the follower has already started running, this will have no effect.
     * </p>
     * 
     * @param follower  The follower to run
     * @param frequency The frequency, in Hz, to run the control loop at
     */
    public void start(Follower<?> follower, int frequency);

    /**
     * Returns whether the follower has finished running.
     * 
     * <p>
     * If the follower was never started, this method will return {@code true}.
     * </p>
     * 
     * @return Whether the follower has finished running
     */
    public boolean isFinished();

    /**
     * Stops running the follower.
     * 
     * <p>
     * If the follower was never started, this method will have no effect.
     * </p>
     */
    public void stop();
}
