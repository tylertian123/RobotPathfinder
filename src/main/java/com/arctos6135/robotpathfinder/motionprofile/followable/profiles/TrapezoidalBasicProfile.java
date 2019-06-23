package com.arctos6135.robotpathfinder.motionprofile.followable.profiles;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.DynamicFollower;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.BasicFollowableMotionProfile;

/**
 * This class uses a {@link TrapezoidalMotionProfile} to move forward in a
 * straight path.
 * <p>
 * Unlike {@link BasicTrajectory}, this class is only capable of handling
 * straight paths with no turns at all. However, it is a lot faster to generate.
 * Therefore, this class should be used in the place of a
 * {@link BasicTrajectory} whenever possible.
 * </p>
 * <p>
 * This class implements {@link DynamicFollowable}, so it is recommended to use
 * a {@link DynamicFollower} for it rather than a normal {@link Follower}.
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public class TrapezoidalBasicProfile extends BasicFollowableMotionProfile<TrapezoidalMotionProfile>
        implements DynamicFollowable<BasicMoment> {

    /**
     * Constructs a new profile with the specifications and distance.
     * 
     * @param specs    The robot specifications
     * @param distance The distance this profile should cover; can be negative for
     *                 backwards motion
     */
    public TrapezoidalBasicProfile(RobotSpecs specs, double distance) {
        profile = new TrapezoidalMotionProfile(specs, distance);
    }

    /**
     * Constructs a new profile with the specifications, distance and facing
     * direction.
     * 
     * @param specs         The robot specifications
     * @param distance      The distance this profile should cover; can be negative
     *                      for backwards motion
     * @param initialFacing The direction the robot is facing
     */
    public TrapezoidalBasicProfile(RobotSpecs specs, double distance, double initialFacing) {
        this(specs, distance);
        this.initialFacing = initialFacing;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(BasicMoment m) {
        profile.update(m.getTime(), m.getPosition(), m.getVelocity(), m.getAcceleration());
    }
}
