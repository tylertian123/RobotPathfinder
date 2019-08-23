package com.arctos6135.robotpathfinder.motionprofile.followable.profiles;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.DynamicFollower;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.motionprofile.DynamicDualMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.TankDriveFollowableMotionProfile;

/**
 * This class uses a {@link TrapezoidalMotionProfile} to move forward in a
 * straight path.
 * <p>
 * Unlike {@link TankDriveTrajectory}, this class is only capable of handling
 * straight paths with no turns at all. However, it is a lot faster to generate.
 * Therefore, this class should be used in the place of a
 * {@link TankDriveTrajectory} whenever possible.
 * </p>
 * <p>
 * This class implements {@link DynamicFollowable}, so it is recommended to use
 * a {@link DynamicFollower} for it rather than a normal {@link Follower}.
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public class TrapezoidalTankDriveProfile
        extends TankDriveFollowableMotionProfile<DynamicDualMotionProfile<TrapezoidalMotionProfile>>
        implements DynamicFollowable<TankDriveMoment>, Cloneable {
    
    /**
     * Constructs a new object of this type without initializing any values.
     * <p>
     * This constructor should only ever be used internally.
     * </p>
     */
    private TrapezoidalTankDriveProfile() {
    }

    /**
     * Constructs a new profile with the specifications and distance.
     * 
     * @param specs    The robot specifications
     * @param distance The distance this profile should cover; can be negative for
     *                 backwards motion
     */
    public TrapezoidalTankDriveProfile(RobotSpecs specs, double distance) {
        profile = new DynamicDualMotionProfile<TrapezoidalMotionProfile>(new TrapezoidalMotionProfile(specs, distance),
                new TrapezoidalMotionProfile(specs, distance));
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
    public TrapezoidalTankDriveProfile(RobotSpecs specs, double distance, double initialFacing) {
        this(specs, distance);
        this.initialFacing = initialFacing;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void update(TankDriveMoment m) {
        profile.updateLeft(m.getTime(), m.getLeftPosition(), m.getLeftVelocity(), m.getLeftAcceleration());
        profile.updateRight(m.getTime(), m.getRightPosition(), m.getRightVelocity(), m.getRightAcceleration());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TrapezoidalTankDriveProfile clone() {
        return copy();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TrapezoidalTankDriveProfile copy() {
        TrapezoidalTankDriveProfile ttdp = new TrapezoidalTankDriveProfile();
        ttdp.profile = profile.copy();
        ttdp.initialFacing = initialFacing;

        return ttdp;
    }
}
