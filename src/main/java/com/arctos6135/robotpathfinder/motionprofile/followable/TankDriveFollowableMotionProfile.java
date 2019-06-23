package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.motionprofile.DualMotionProfile;

/**
 * This abstract class is a wrapper around a {@link DualMotionProfile} that also
 * implements {@link com.arctos6135.robotpathfinder.follower.Followable
 * Followable}.
 * <p>
 * It contains an internal dual motion profile, and uses it to generate
 * {@link TankDriveMoment}s for a given time.
 * </p>
 * 
 * @param <T> The type of {@link DualMotionProfile} used
 * @author Tyler Tian
 * @since 3.0.0
 * @see com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalTankDriveProfile
 *      TrapezoidalTankDriveProfile
 */
public abstract class TankDriveFollowableMotionProfile<T extends DualMotionProfile<?>>
        extends FollowableMotionProfile<TankDriveMoment> {

    protected T profile;

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
        boolean backwards = profile.isReversed();
        return new TankDriveMoment(profile.leftPosition(t), profile.rightPosition(t), profile.leftVelocity(t),
                profile.rightVelocity(t), profile.leftAcceleration(t), profile.rightAcceleration(t),
                backwards ? -initialFacing : initialFacing, t, initialFacing, backwards);
    }
}
