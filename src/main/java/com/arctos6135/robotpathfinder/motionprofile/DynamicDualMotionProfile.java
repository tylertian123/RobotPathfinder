package com.arctos6135.robotpathfinder.motionprofile;

/**
 * This class is an extension of {@link DualMotionProfile} for
 * {@link DynamicMotionProfile}s.
 * <p>
 * In addition to the methods provided by {@link DualMotionProfile}, this class
 * also has two more methods,
 * {@link #updateLeft(double, double, double, double)} and
 * {@link #updateRight(double, double, double, double)}, which call the
 * {@link DynamicMotionProfile#update(double, double, double, double) update()}
 * method of the internal motion profiles.
 * </p>
 * <p>
 * As always, calling the update methods mentioned above will modify the object
 * itself. To use the object multiple times, consider using {@link #copy()} to
 * create an identical copy, while maintaining the original object.
 * </p>
 * 
 * @author Tyler Tian
 * @param <T> The type of {@link MotionProfile} used; must be a subclass of
 *            {@link DynamicMotionProfile}
 * @see DualMotionProfile
 * @see DynamicMotionProfile
 * @since 3.0.0
 */
public class DynamicDualMotionProfile<T extends DynamicMotionProfile> extends DualMotionProfile<T> {

    /**
     * Constructs a new {@link DynamicDualMotionProfile} from two regular motion
     * profiles.
     * 
     * @param left  The left wheel's motion profile
     * @param right The right wheel's motion profile
     */
    public DynamicDualMotionProfile(T left, T right) {
        super(left, right);
    }

    /**
     * Calls the {@link DynamicMotionProfile#update(double, double, double, double)
     * update()} method of the left motion profile.
     * <p>
     * As always, calling this method will modify the object itself. To use the
     * object multiple times, consider using {@link #copy()} to create an identical
     * copy, while maintaining the original object.
     * </p>
     * <p>
     * For more information, see the documentation for
     * {@link DynamicMotionProfile#update(double, double, double, double)}.
     * <p>
     * 
     * @param currentTime  The current time
     * @param currentPos   The current position
     * @param currentVel   The current velocity
     * @param currentAccel The current acceleration
     * @return Whether the updated motion profile has to overshoot
     */
    public boolean updateLeft(double currentTime, double currentPos, double currentVel, double currentAccel) {
        return leftProfile.update(currentTime, currentPos, currentVel, currentAccel);
    }

    /**
     * Calls the {@link DynamicMotionProfile#update(double, double, double, double)
     * update()} method of the right motion profile.
     * <p>
     * As always, calling this method will modify the object itself. To use the
     * object multiple times, consider using {@link #copy()} to create an identical
     * copy, while maintaining the original object.
     * </p>
     * <p>
     * For more information, see the documentation for
     * {@link DynamicMotionProfile#update(double, double, double, double)}.
     * <p>
     * 
     * @param currentTime  The current time
     * @param currentPos   The current position
     * @param currentVel   The current velocity
     * @param currentAccel The current acceleration
     * @return Whether the updated motion profile has to overshoot
     */
    public boolean updateRight(double currentTime, double currentPos, double currentVel, double currentAccel) {
        return rightProfile.update(currentTime, currentPos, currentVel, currentAccel);
    }

    /**
     * Creates an identical deep copy of this {@link DynamicDualMotionProfile}
     * object.
     * 
     * @return An identical copy of this object
     */
    @SuppressWarnings("unchecked")
    public DynamicDualMotionProfile<T> copy() {
        // This unchecked cast should always work, unless the copy() methods weren't
        // implemented correctly, in which case a ClassCastException would be
        // appropriate anyways
        return new DynamicDualMotionProfile<T>((T) leftProfile.copy(), (T) rightProfile.copy());
    }
}
