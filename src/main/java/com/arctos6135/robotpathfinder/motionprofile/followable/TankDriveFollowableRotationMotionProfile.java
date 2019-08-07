package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.math.MathUtils;
import com.arctos6135.robotpathfinder.motionprofile.DualMotionProfile;

/**
 * This abstract class is a wrapper around a {@link DualMotionProfile} that also
 * implements {@link com.arctos6135.robotpathfinder.follower.Followable
 * Followable} and used for rotating in place.
 * <p>
 * It contains an internal dual motion profile, and uses it to generate
 * {@link TankDriveMoment}s for a given time. The generated moments will make
 * the robot spin in place to reach a specified angle.
 * </p>
 * 
 * @author Tyler Tian
 * @param <T> The type of {@link DualMotionProfile} used
 * @see com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveRotationProfile
 *      TrapezoidalTankDriveRotationProfile
 * @since 3.0.0
 */
public abstract class TankDriveFollowableRotationMotionProfile<T extends DualMotionProfile<?>>
        extends FollowableMotionProfile<TankDriveMoment> {

    protected T profile;

    /**
     * The width of the base plate of the robot. Must be set by the constructors of
     * implementing classes!!
     */
    protected double baseWidth;

    protected double initialFacing = Math.PI / 2;

    /**
     * {@inheritDoc}
     */
    @Override
    public double totalTime() {
        return profile.totalTime();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TankDriveMoment get(double t) {
        double ld = profile.leftPosition(t);
        double rd = profile.rightPosition(t);

        // The overall angle rotated is equivalent to the angle caused by the right
        // wheel moving and the angle caused by the left wheel moving combined.
        // Since the angle is in radians, simply divide the arc length by the radius to
        // get the angle.
        // The left distance is negative since it moving forward will actually turn the
        // robot clockwise.
        double currentAngle = MathUtils.restrictAngle(rd / baseWidth - ld / baseWidth + initialFacing);
        return new TankDriveMoment(ld, rd, profile.leftVelocity(t), profile.rightVelocity(t),
                profile.leftAcceleration(t), profile.rightAcceleration(t), currentAngle, t, initialFacing, false);
    }
}
