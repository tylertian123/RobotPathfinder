package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

/**
 * This is the base class for all the dynamic follower classes. It inherits from
 * {@link Follower}.
 * <p>
 * See {@link Follower} for the basic definition of a "follower". The
 * {@link DynamicFollower} is a special type of follower that follows
 * {@link DynamicFollowable}s instead of regular {@link Followables}. They will
 * periodically call the {@link DynamicFollowable}'s
 * {@link DynamicFollowable#update(Moment) update()} method to re-generate it,
 * with an interval between calls specified in the constructor.
 * </p>
 * <p>
 * See the documentation for {@link Follower} for usage instructions.
 * </p>
 * 
 * @param <T> The type of moment used by this {@link DynamicFollower}; must be a
 *            subclass of {@link Moment}
 * @author Tyler Tian
 * @since 3.0.0
 * @see Follower
 * @see DynamicFollowable
 */
abstract public class DynamicFollower<T extends Moment> extends Follower<T> {

    protected double updateDelay = Double.NaN;
    protected double lastUpdateTime;

    /**
     * This method must be overridden by a subclass with an implementation. It
     * should call the target's {@link DynamicFollowable#update(Moment)} method with
     * real life data collected automatically from the sensors.
     * <p>
     * Note that to accomplish this, the target variable may need to be casted into
     * a {@link DynamicFollowable}. To avoid a {@link java.lang.ClassCastException},
     * all constructors should only take {@link DynamicFollowable}s as opposed to
     * regular {@link Followable}s.
     * </p>
     */
    abstract protected void _update();

    /**
     * Forces the {@link DynamicFollower} to call the
     * {@link DynamicFollowable#update(Moment) update()} method of its target.
     * <p>
     * This method also resets the time of the last update; therefore, regardless of
     * when this method was called, the next time
     * {@link DynamicFollowable#update(Moment) update()} is called will be exactly
     * after an interval the length of the update delay.
     * </p>
     */
    public void update() {
        _update();
        lastUpdateTime = timer.getTimestamp();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void initialize() {
        super.initialize();
        lastUpdateTime = timer.getTimestamp();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void run() {
        super.run();

        double currentTime = timer.getTimestamp();
        if (currentTime - lastUpdateTime >= updateDelay) {
            update();
            lastUpdateTime = currentTime;
        }
    }

    /**
     * Retrieves the time between two updates.
     * <p>
     * The {@link DynamicFollower} periodically calls the target's
     * {@link DynamicFollowable#update(Moment) update()} method. This is the amount
     * of time that must elapse between two calls.
     * </p>
     * 
     * @return The update delay
     */
    public double getUpdateDelay() {
        return updateDelay;
    }

    /**
     * Sets the time between two updates.
     * <p>
     * The {@link DynamicFollower} periodically calls the target's
     * {@link DynamicFollowable#update(Moment) update()} method. This is the amount
     * of time that must elapse between two calls.
     * </p>
     * 
     * @param updateDelay The update delay; if set to {@code NaN}, updating will be
     *                    disabled
     */
    public void setUpdateDelay(double updateDelay) {
        this.updateDelay = updateDelay;
    }

    /**
     * Disables updating.
     * <p>
     * Calling this method will completely disable updating. It is equivalent to
     * calling {@link #setUpdateDelay(double)} with {@code NaN}.
     * <p>
     */
    public void disableUpdates() {
        updateDelay = Double.NaN;
    }
}
