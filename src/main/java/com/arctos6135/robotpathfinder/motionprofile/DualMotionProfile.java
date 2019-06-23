package com.arctos6135.robotpathfinder.motionprofile;

/**
 * This class is a wrapper around two {@link MotionProfile}s.
 * <p>
 * This class is commonly used by tank drive motion profiles. It contains two
 * {@link MotionProfile}s, one for the left wheel and one for the right wheel.
 * It exposes a number of methods for working with these profiles individually
 * and together.
 * </p>
 * 
 * @param <T> The type of {@link MotionProfile} used
 * @author Tyler Tian
 * @since 3.0.0
 * @see MotionProfile
 */
public class DualMotionProfile<T extends MotionProfile> {

    protected T leftProfile, rightProfile;

    /**
     * Constructs a new {@link DualMotionProfile} from two regular motion profiles.
     * 
     * @param left  The left wheel's motion profile
     * @param right The right wheel's motion profile
     */
    public DualMotionProfile(T left, T right) {
        leftProfile = left;
        rightProfile = right;
    }

    /**
     * Retrieves the total time it takes to complete this {@link DualMotionProfile}.
     * <p>
     * This is equal to the greater of the two motion profiles' total times.
     * Equivalent to
     * {@code Math.max(leftProfile.totalTime(), rightProfile.totalTime())}.
     * </p>
     * 
     * @return The total time it takes to complete this {@link DualMotionProfile}.
     */
    public double totalTime() {
        return Math.max(leftProfile.totalTime(), rightProfile.totalTime());
    }

    /**
     * Retrieves whether this {@link DualMotionProfile} is reversed (driving
     * backwards).
     * <p>
     * This will only return true if both the left and right motion profiles are
     * reversed. Equivalent to
     * {@code leftProfile.isReversed() && rightProfile.isReversed()}.
     * </p>
     * 
     * @return Whether this dual motion profile is reversed
     */
    public boolean isReversed() {
        return leftProfile.isReversed() && rightProfile.isReversed();
    }

    /**
     * Retrieves the position at the specified time of the left motion profile.
     * 
     * @param t The time
     * @return The position of the left profile at the specified time
     */
    public double leftPosition(double t) {
        return leftProfile.position(t);
    }

    /**
     * Retrieves the position at the specified time of the right motion profile.
     * 
     * @param t The time
     * @return The position of the right profile at the specified time
     */
    public double rightPosition(double t) {
        return rightProfile.position(t);
    }

    /**
     * Retrieves the velocity at the specified time of the left motion profile.
     * 
     * @param t The time
     * @return The velocity of the left profile at the specified time
     */
    public double leftVelocity(double t) {
        return leftProfile.velocity(t);
    }

    /**
     * Retrieves the velocity at the specified time of the right motion profile.
     * 
     * @param t The time
     * @return The velocity of the right profile at the specified time
     */
    public double rightVelocity(double t) {
        return rightProfile.velocity(t);
    }

    /**
     * Retrieves the acceleration at the specified time of the left motion profile.
     * 
     * @param t The time
     * @return The acceleration of the left profile at the specified time
     */
    public double leftAcceleration(double t) {
        return leftProfile.acceleration(t);
    }

    /**
     * Retrieves the acceleration at the specified time of the right motion profile.
     * 
     * @param t The time
     * @return The acceleration of the right profile at the specified time
     */
    public double rightAcceleration(double t) {
        return rightProfile.acceleration(t);
    }
}
