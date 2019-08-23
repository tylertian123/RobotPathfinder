package com.arctos6135.robotpathfinder.motionprofile.followable;

import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;

/**
 * This abstract class is a wrapper around a {@link MotionProfile} that also
 * implements {@link Followable}.
 * <p>
 * It contains an internal motion profile, and uses it to generate
 * {@link BasicMoment}s for a given time.
 * </p>
 * 
 * @author Tyler Tian
 * @param <T> The type of {@link MotionProfile} used
 * @see com.arctos6135.robotpathfinder.motionprofile.followable.profiles.TrapezoidalBasicProfile
 *      TrapezoidalBasicProfile
 * @since 3.0.0
 */
public abstract class BasicFollowableMotionProfile<T extends MotionProfile>
        extends FollowableMotionProfile<BasicMoment> {

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
    public BasicMoment get(double t) {
        double d = profile.position(t);
        double v = profile.velocity(t);
        double a = profile.acceleration(t);

        boolean backwards = profile.isReversed();

        return new BasicMoment(d, v, a, backwards ? -initialFacing : initialFacing, t, initialFacing, backwards);
    }
}
