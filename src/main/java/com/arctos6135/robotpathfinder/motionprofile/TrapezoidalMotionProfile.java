package com.arctos6135.robotpathfinder.motionprofile;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.math.MathUtils;

/**
 * This class represents a trapezoidal motion profile.
 * <p>
 * In a trapezoidal motion profile, velocity and acceleration are limited to a
 * maximum value. However, jerk (the derivative of acceleration) is not, which
 * means that the acceleration can jump around instantaneously.
 * </p>
 * <p>
 * At the start of a trapezoidal motion profile, the acceleration will typically
 * be set to the maximum, causing the velocity to increase linearly. Once the
 * maximum velocity is reached, the acceleration will be set to 0, causing the
 * velocity to become a horizontal line. Near the end, the acceleration will be
 * set to the negative of the maximum, causing the velocity to decrease
 * linearly. As a result, the graph of velocity vs. time looks like a trapezoid,
 * hence the name.
 * </p>
 * <p>
 * This motion profile is {@link DynamicMotionProfile dynamic}.
 * </p>
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public class TrapezoidalMotionProfile implements DynamicMotionProfile, Cloneable {

    protected double initVel;
    protected double initDist, initTime;

    protected double distance;
    protected double maxAcl, maxVel;
    protected double cruiseVel;

    protected double tAccel, tCruise, tTotal;
    protected double accelDist, cruiseDist;

    protected boolean reverse = false;

    protected final RobotSpecs specs;

    @Override
    public String toString() {
        return "\u001b[92m[\u001b[4mTMP" + this.hashCode() + "\u001b[24m with initVel=" + initVel + ", initDist="
                + initDist + ", initTime=" + initTime + ", distance=" + distance + ", maxAcl=" + maxAcl + ", maxVel="
                + maxVel + ", cruiseVel=" + cruiseVel + ", tAccel=" + tAccel + ", tCruise=" + tCruise + ", tTotal="
                + tTotal + ", accelDist=" + accelDist + ", cruiseDist=" + cruiseDist + ", reverse=" + reverse
                + "]\u001b[0m";
    }

    /**
     * Constructs a new object of this type without initializing any values.
     * <p>
     * This constructor should only ever be used internally.
     * </p>
     */
    private TrapezoidalMotionProfile(RobotSpecs specs) {
        this.specs = specs;
    }

    /**
     * Constructs a new trapezoidal motion profile with a set distance.
     * 
     * @param specs The specifications of the robot (max velocity and acceleration)
     * @param dist  The distance this motion profile should cover; can be negative
     *              for backwards motion
     */
    public TrapezoidalMotionProfile(RobotSpecs specs, double dist) {
        this.specs = specs;
        construct(specs, dist, 0);
    }

    /**
     * Constructs a new trapezoidal motion profile with a set distance and initial
     * velocity.
     * 
     * @param specs   The specifications of the robot (max velocity and
     *                acceleration)
     * @param dist    The distance this motion profile should cover; can be negative
     *                for backwards motion
     * @param initVel The velocity of the robot at t=0
     */
    public TrapezoidalMotionProfile(RobotSpecs specs, double dist, double initVel) {
        this.specs = specs;
        construct(specs, dist, initVel);
    }

    /**
     * Constructs the motion profile.
     * <p>
     * Used by the constructor and {@link #update(double, double, double, double)}.
     * </p>
     * 
     * @param maxVel   The max velocity
     * @param maxAccel The max acceleration
     * @param dist     The distance to cover
     * @param initVel  The initial velocity
     * @return Whether or not the motion profile overshoots
     */
    private boolean construct(RobotSpecs specs, double dist, double initVel) {
        System.out.printf(
                "\u001b[94mconstructing\u001b[0m %s \u001b[94mwith maxVel=%f, maxAccel=%f, dist=%f and initVel=%f\u001b[0m\n",
                this.toString(), specs.getMaxVelocity(), specs.getMaxAcceleration(), dist, initVel);
        if (dist < 0) {
            reverse = true;
            dist = -dist;
            initVel = -initVel;
        }
        distance = dist;
        maxAcl = specs.getMaxAcceleration();
        maxVel = Math.max(specs.getMaxVelocity(), Math.abs(initVel));
        this.initVel = initVel;

        // // Use MathUtils functions to compare floats
        // if (MathUtils.floatGt(Math.abs(initVel), maxVel)) {
        // throw new IllegalArgumentException("Initial velocity too high!");
        // }

        // Calculate the distance covered when accelerating and decelerating
        // Formula is derived from the kinematic formula relating velocities, distance
        // and acceleration, and the fact that dAccel + dDecel = dist
        // Assumes there is no maximum cap on velocity
        double dAccel = dist / 2 - initVel * initVel / (4 * maxAcl);
        double dDecel;
        // If the acceleration distance is less than 0, the distance is not enough to
        // decelerate back to 0
        // Change the maximum acceleration so that we can
        boolean overshoot = false;
        if (dAccel < 0) {
            // Unless the distance left is 0
            // In which case we promptly give up and call it a day
            if (dist == 0) {
                System.err.println("WTF");
                tAccel = tCruise = tTotal = 0;
                return true;
            }
            // This value should make dAccel equal to 0
            this.maxAcl = initVel * initVel / (2 * dist);
            dDecel = dist;
            overshoot = true;
        } else {
            dDecel = dist - dAccel;
        }
        // Calculate cruise velocity
        double vc = Math.sqrt(2 * maxAcl * dDecel);
        cruiseVel = Math.min(vc, maxVel);

        // Calculate acceleration time
        tAccel = (cruiseVel - initVel) / maxAcl;
        // Re-calculate the acceleration distance
        // This is needed because the first result does not take into account the actual
        // max velocity
        // First kinematic formula
        accelDist = tAccel * tAccel * maxAcl * 0.5 + initVel * tAccel;
        // Calculate the deceleration time
        double tDecel = cruiseVel / maxAcl;
        // Re-calculate the deceleration distance
        double decelDist = tDecel * tDecel * maxAcl * 0.5;

        // Calculate the cruise distance
        cruiseDist = dist - accelDist - decelDist;
        // Calculate the cruise time
        tCruise = cruiseDist / cruiseVel;
        // tTotal is the total time in the range of this motion profile
        // It does not include initTime
        tTotal = tAccel + tCruise + tDecel;
        System.out.println("\u001b[93mAfter construction: " + this);
        return overshoot;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double totalTime() {
        // Add initTime to tTotal to get the absolute time
        return tTotal + initTime;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean isReversed() {
        return reverse;
    }

    /**
     * {@inheritDoc}
     * <p>
     * Note: If {@link #update(double, double, double, double) update()} was
     * previously called with a nonzero time value, this method will throw an
     * {@link IllegalArgumentException} if the time is less than the time passed to
     * {@link #update(double, double, double, double) update()}.
     * </p>
     * 
     * @throws IllegalArgumentException If the time is out of range
     */
    @Override
    public double position(double time) {
        if (MathUtils.floatLt(time, initTime)) {
            throw new IllegalArgumentException(
                    String.format("Time out of range (%f not in [%f, %f])!", time, initTime, initTime + tTotal));
        }
        time -= initTime;
        double result = 0;
        // When accelerating
        if (time < tAccel) {
            result = time * time * maxAcl * 1 / 2 + initVel * time;
        }
        // When cruising
        else if (time < tAccel + tCruise) {
            // The distance is the distance covered during acceleration and rest of the time
            // multiplied by the cruise velocity
            result = accelDist + (time - tAccel) * cruiseVel;
        }
        // When decelerating
        else if (MathUtils.floatLtEq(time, tTotal)) {
            // The distance is the distance covered during acceleration and cruising, plus
            // the distance covered during deceleration,
            // which can be solved using the third kinematic formula
            double t = time - tAccel - tCruise;
            result = accelDist + cruiseDist + t * cruiseVel - t * t * maxAcl * 0.5;
        } else {
            throw new IllegalArgumentException(String.format("Time out of range (%f not in [%f, %f])!", time + initTime,
                    initTime, initTime + tTotal));
        }
        return (reverse ? -result : result) + initDist;
    }

    /**
     * {@inheritDoc}
     * <p>
     * Note: If {@link #update(double, double, double, double) update()} was
     * previously called with a nonzero time value, this method will throw an
     * {@link IllegalArgumentException} if the time is less than the time passed to
     * {@link #update(double, double, double, double) update()}.
     * </p>
     * 
     * @throws IllegalArgumentException If the time is out of range
     */
    @Override
    public double velocity(double time) {
        if (MathUtils.floatLt(time, initTime)) {
            throw new IllegalArgumentException(
                    String.format("Time out of range (%f not in [%f, %f])!", time, initTime, initTime + tTotal));
        }
        time -= initTime;
        double result = 0;
        // When accelerating
        if (time < tAccel) {
            // The velocity is just the time multiplied by the acceleration
            result = time * maxAcl + initVel;
        }
        // When cruising
        else if (time < tAccel + tCruise) {
            // The velocity is the cruise velocity
            result = cruiseVel;
        }
        // When decelerating
        else if (MathUtils.floatLtEq(time, tTotal)) {
            // The velocity is the cruise velocity minus the acceleration times the time
            // decelerating
            result = cruiseVel - (time - tAccel - tCruise) * maxAcl;
        } else {
            throw new IllegalArgumentException(String.format("Time out of range (%f not in [%f, %f])!", time + initTime,
                    initTime, initTime + tTotal));
        }
        return reverse ? -result : result;
    }

    /**
     * {@inheritDoc}
     * <p>
     * Note: If {@link #update(double, double, double, double) update()} was
     * previously called with a nonzero time value, this method will throw an
     * {@link IllegalArgumentException} if the time is less than the time passed to
     * {@link #update(double, double, double, double) update()}.
     * </p>
     * 
     * @throws IllegalArgumentException If the time is out of range
     */
    @Override
    public double acceleration(double time) {
        if (MathUtils.floatLt(time, initTime)) {
            throw new IllegalArgumentException(
                    String.format("Time out of range (%f not in [%f, %f])!", time, initTime, initTime + tTotal));
        }
        time -= initTime;
        double result = 0;
        // When accelerating
        if (time < tAccel) {
            result = maxAcl;
        }
        // When cruising
        else if (time < tAccel + tCruise) {
            result = 0;
        }
        // When decelerating
        else if (MathUtils.floatLtEq(time, tTotal)) {
            result = -maxAcl;
        } else {
            throw new IllegalArgumentException(String.format("Time out of range (%f not in [%f, %f])!", time + initTime,
                    initTime, initTime + tTotal));
        }
        return reverse ? -result : result;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean update(double currentTime, double currentDist, double currentVel, double currentAccel) {
        System.out.printf(
                "%s \u001b[91mupdating with currentTime=%f, currentDist=%f, currentVel=%f and currentAccel=%f\u001b[0m\n",
                this.toString(), currentTime, currentDist, currentVel, currentAccel);
        initTime = currentTime;
        double prevInitDist = initDist;
        initDist = currentDist;
        return construct(specs, (reverse ? -distance : distance) + prevInitDist - currentDist, currentVel);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TrapezoidalMotionProfile clone() {
        return copy();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public TrapezoidalMotionProfile copy() {
        TrapezoidalMotionProfile profile = new TrapezoidalMotionProfile(specs);
        profile.initVel = initVel;
        profile.initDist = initDist;
        profile.initTime = initTime;

        profile.distance = distance;
        profile.maxAcl = maxAcl;
        profile.maxVel = maxVel;
        profile.cruiseVel = cruiseVel;

        profile.tAccel = tAccel;
        profile.tCruise = tCruise;
        profile.tTotal = tTotal;

        profile.accelDist = accelDist;
        profile.cruiseDist = cruiseDist;

        profile.reverse = reverse;

        return profile;
    }
}
