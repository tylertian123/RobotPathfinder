package com.arctos6135.robotpathfinder.motionprofile.followable.profiles;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.follower.DynamicFollowable;
import com.arctos6135.robotpathfinder.follower.DynamicFollower;
import com.arctos6135.robotpathfinder.follower.Follower;
import com.arctos6135.robotpathfinder.motionprofile.DynamicDualMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.TrapezoidalMotionProfile;
import com.arctos6135.robotpathfinder.motionprofile.followable.TankDriveFollowableRotationMotionProfile;

/**
 * This class uses a {@link TrapezoidalMotionProfile} to turn in place.
 * <p>
 * Unlike {@link TankDriveTrajectory}, this class is only capable of turning in
 * place, with no other motions at all. However, it is a lot faster to generate.
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
public class TrapezoidalTankDriveRotationProfile
        extends TankDriveFollowableRotationMotionProfile<DynamicDualMotionProfile<TrapezoidalMotionProfile>>
        implements DynamicFollowable<TankDriveMoment> {

    /**
     * Constructs a new profile with the specifications and angle.
     * 
     * @param specs The robot specifications
     * @param angle The angle to rotate for (follows the unit circle)
     */
    public TrapezoidalTankDriveRotationProfile(RobotSpecs specs, double angle) {
        baseWidth = specs.getBaseWidth();
        profile = new DynamicDualMotionProfile<TrapezoidalMotionProfile>(
                // As there are two wheels turning, each only need to go half the distance
                // To put it another way, the radius is now half of the base width instead of
                // the full base width
                // Negate the final distance for left because it turns in reverse
                new TrapezoidalMotionProfile(specs, -angle * baseWidth / 2),
                new TrapezoidalMotionProfile(specs, angle * baseWidth / 2));
    }

    /**
     * Constructs a new profile with the specifications, angle and initial facing
     * direction.
     * 
     * @param specs         The robot specifications
     * @param angle         The angle to rotate for (follows the unit circle)
     * @param initialFacing The direction the robot is facing at the beginning of
     *                      the rotation
     */
    public TrapezoidalTankDriveRotationProfile(RobotSpecs specs, double angle, double initialFacing) {
        this(specs, angle);
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
}
