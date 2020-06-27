package com.arctos6135.robotpathfinder.motionprofile;

/**
 * This interface is an extension of {@link MotionProfile} and defines the basic
 * requirements for a dynamic motion profile.
 * <p>
 * See {@link MotionProfile} for the basic definition of a motion profile. In
 * addition to the constraints specified by {@link MotionProfile}, a class must
 * also provide an additional method
 * {@link #update(double, double, double, double)}. This method takes
 * information about the <em>real life</em> conditions of the robot, and
 * updates/re-generates the motion profile to better match those conditions.
 * </p>
 * <p>
 * Note that calling the method {@link #update(double, double, double, double)}
 * will modify the motion profile object itself. If the dynamic motion profile
 * needs to be reused, the {@link #copy()} method can be used to create an
 * identical copy, while maintaining the original object.
 * </p>
 * 
 * @author Tyler Tian
 * @see MotionProfile
 * @since 3.0.0
 */
public interface DynamicMotionProfile extends MotionProfile {
    /**
     * This method takes information about the <em>real life</em> conditions of the
     * robot, and updates/re-generates the motion profile to better match those
     * conditions.
     * <p>
     * Sometimes, in order to match the conditions given, the motion profile must be
     * modified such that the final position is different than the position
     * initially specified (since there may not be enough distance to fully slow
     * down). This is referred to as "overshooting". If the updated motion profile
     * overshoots, then the method will return true.
     * </p>
     * <p>
     * Note that calling this method will modify the motion profile object itself.
     * If the dynamic motion profile needs to be reused, the {@link #copy()} method
     * can be used to create an identical copy, while maintaining the original
     * object.
     * </p>
     * <p>
     * For more information, see the class Javadoc for this class.
     * </p>
     * 
     * @param currentTime  The current time
     * @param currentPos   The current position
     * @param currentVel   The current velocity
     * @param currentAccel The current acceleration
     * @return Whether the updated motion profile has to overshoot
     */
    public boolean update(double currentTime, double currentPos, double currentVel, double currentAccel);

    /**
     * Creates an identical deep copy of this {@link DynamicMotionProfile} object.
     * 
     * @return An identical copy of this object
     */
    public DynamicMotionProfile copy();
}
