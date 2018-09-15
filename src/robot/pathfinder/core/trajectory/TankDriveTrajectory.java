package robot.pathfinder.core.trajectory;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.path.Path;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;

/**
 * A class that represents a trajectory (motion profile) for a tank drive robot. A trajectory not only defines the points that a 
 * robot will path through, it also provides information about the velocity, acceleration and direction. Most 
 * importantly, for any given time, it can provide information about the robot's whereabouts and other information. 
 * <p>
 * Trajectories generated by this class assume a hypothetical robot that has limited speed and acceleration,
 * but unlimited jerk. Unlike {@link BasicTrajectory}, this class breaks the overall movement down to the individual
 * movements of the left and right side wheels, and takes into account the slowing down when turning. 
 * </p>
 * <p>
 * This class cannot generate its own base trajectory; rather, it takes an already generated {@link BasicTrajectory}
 * (with {@link TrajectoryParams#isTank} {@code true}) and uses it as a base to construct itself.
 * </p>
 * @see BasicTrajectory
 * @author Tyler Tian
 *
 */
public class TankDriveTrajectory implements Trajectory {
	
	//The internal path that this trajectory is based on
	Path path;
	//"Moments" are generated for left and right separately
	TankDriveMoment[] moments;
	Vec2D[] headingVectors;
	
	RobotSpecs specs;
	TrajectoryParams params;
	
	double initialFacing;
	
	/**
	 * Generates a new {@link TankDriveTrajectory} based on the {@link BasicTrajectory} provided.
	 * <p>
	 * The {@link BasicTrajectory} must be generated as a tank drive trajectory; that is, the field
	 * {@link TrajectoryParams#isTank} must be set to true during generation. Otherwise, an {@link IllegalArgumentException}
	 * will be thrown.
	 * </p>
	 * @param traj The base trajectory
	 */
	public TankDriveTrajectory(BasicTrajectory traj) {
		if(!traj.isTank()) {
			throw new IllegalArgumentException("Base trajectory is not generated with tank drive");
		}
		
		//Initialize and copy some fields
		BasicMoment[] trajMoments = traj.getMoments();
		//Initialize moments array
		moments = new TankDriveMoment[trajMoments.length];
		moments[0] = new TankDriveMoment(0, 0, 0, 0, 0, 0, trajMoments[0].getHeading(), 0);
		
		specs = traj.getRobotSpecs();
		params = traj.getGenerationParams();
		headingVectors = traj.headingVectors;
		initialFacing = traj.initialFacing;
		
		path = traj.getPath();
		RobotSpecs specs = traj.getRobotSpecs();
		double baseRadius = specs.getBaseWidth() / 2;
		double maxVel = specs.getMaxVelocity();
		path.setBaseRadius(traj.getRobotSpecs().getBaseWidth() / 2);
		
		//Numerical integration is used
		//This keeps track of where the wheels were in the last interation
		Vec2D[] initPos = path.wheelsAt(0);
		Vec2D prevLeft = initPos[0], prevRight = initPos[1];
		
		for(int i = 1; i < trajMoments.length; i ++) {
			//First find where the wheels are at this moment and integrate the length
			Vec2D[] wheelPos = path.wheelsAt(traj.pathT[i]);
			double dxLeft = prevLeft.distTo(wheelPos[0]);
			double dxRight = prevRight.distTo(wheelPos[1]);
			double dt = trajMoments[i].getTime() - trajMoments[i - 1].getTime();
			
			prevLeft = wheelPos[0];
			prevRight = wheelPos[1];
			
			//Find out the velocity of the two wheels
			double vel = trajMoments[i].getVelocity();
			/*
			 * The formula for velocity is derived as follows:
			 * Start with the equation:
			 * 1. w = v/r, where w is the angular velocity, r is the radius of the path, and v is the velocity
			 * 
			 * 1. From the equation we can get v = wr
			 * 2. Let b represent the base radius; then, the radius for the left wheel is r - b, the radius for
			 * the right wheel is r + b
			 * 3. Substitute r: v1 = w(r - b), v2 = w(r + b), where v1 is the left wheel velocity, v2 is the right
			 * wheel velocity
			 * 4. Substitute w: v1 = (v/r) (r - b), v2 = (v/r) (r + b)
			 * 5. Distribute: v1 = v - (v/r) * b, v2 = v + (v/r) * b
			 * 
			 * The nice thing about using path radius to figure out the velocity is now we can have negative
			 * velocities when the turn is too tight and the wheel has to move backwards, unlike the distance
			 * difference which is always positive.
			 */
			//Use MathUtils.clampAbs() to do a sanity check
			//The calculated velocity should never be greater than the maximum (this can be proven algebraically)
			//But add a check just in case
			double leftVelocity = MathUtils.clampAbs(vel - vel / traj.pathRadius[i] * baseRadius, maxVel);
			double rightVelocity = MathUtils.clampAbs(vel + vel / traj.pathRadius[i] * baseRadius, maxVel);
			//As described, a negative velocity means the wheel has to move backwards
			//So negate the distance difference
			if(leftVelocity < 0) {
				dxLeft = -dxLeft;
			}
			if(rightVelocity < 0) {
				dxRight = -dxRight;
			}
			
			moments[i] = new TankDriveMoment();
			//Integrate position
			moments[i].setLeftPosition(moments[i - 1].getLeftPosition() + dxLeft);
			moments[i].setRightPosition(moments[i - 1].getRightPosition() + dxRight);
			moments[i].setLeftVelocity(leftVelocity);
			moments[i].setRightVelocity(rightVelocity);
			//Derive velocity to get acceleration
			moments[i - 1].setLeftAcceleration((moments[i].getLeftVelocity() - moments[i - 1].getLeftVelocity()) / dt);
			moments[i - 1].setRightAcceleration((moments[i].getRightVelocity() - moments[i - 1].getRightVelocity()) / dt);
			moments[i].setTime(trajMoments[i].getTime());
			moments[i].setHeading(trajMoments[i].getHeading());
			moments[i].setInitialFacing(initialFacing);
		}
	}
	/**
	 * Creates a tank drive trajectory with the specified robot specifications and trajectory parameters.
	 * @param specs A {@link RobotSpecs} object with the robot's properties, such as max speed and acceleration
	 * @param params A {@link TrajectoryParams} object with the parameters for this trajectory
	 */
	public TankDriveTrajectory(RobotSpecs specs, TrajectoryParams params) {
		this(new BasicTrajectory(specs, params));
	}
	
	/*
	 * This constructor is used internally by the mirrorLeftRight, mirrorFrontBack and retrace methods.
	 * It requires pre-generated moments so it's not visible to the world.
	 */
	protected TankDriveTrajectory(TankDriveMoment[] moments, Path path, RobotSpecs specs, TrajectoryParams params) {
		this.moments = moments;
		this.path = path;
		this.specs = specs;
		this.params = params;
		this.initialFacing = moments[0].getInitialFacing();
		
		headingVectors = new Vec2D[moments.length];
		for(int i = 0; i < moments.length; i ++) {
			headingVectors[i] = new Vec2D(Math.cos(moments[i].getHeading()), Math.sin(moments[i].getHeading()));
		}
	}

	
	/**
	 * Retrieves the interal {@link Path} followed by this trajectory.
	 * @return The {@link Path} followed by this trajectory
	 */
	public Path getPath() {
		return path;
	}
	
	/**
	 * Retrieves the array of moment objects generated by this trajectory. 
	 * @return The array of moment objects generated by this trajectory
	 */
	public TankDriveMoment[] getMoments() {
		return moments;
	}
	
	/**
	 * Retrieves the total time it takes to complete this trajectory.
	 * @return The total amount of time it takes to drive this trajectory
	 */
	public double totalTime() {
		return moments[moments.length - 1].getTime();
	}
	
	/**
	 * Returns the {@link RobotSpecs} object used to generate this trajectory.
	 * @return The {@link RobotSpecs} object used to generate this trajectory
	 */
	public RobotSpecs getRobotSpecs() {
		return specs;
	}
	/**
	 * Returns the {@link TrajectoryParams} object used to generate this trajectory.
	 * @return The {@link TrajectoryParams} object used to generate this trajectory
	 */
	public TrajectoryParams getGenerationParams() {
		return params;
	}
	
	/**
	 * Retrieves the moment object associated with the specified time. If there is no moment object with the 
	 * same time as the time specified, the result will be approximated with linear interpolation.
	 * <p>
	 * Moment objects contain information about the position, velocity, acceleration and direction of a robot
	 * at a certain time. They're returned by trajectories when querying a specific time. For more information,
	 * see the {@link TankDriveMoment} class.
	 * </p>
	 * <p>
	 * Note that all moment objects are cloned before being returned, therefore it is safe to modify a moment.
	 * </p>
	 * @param t The time
	 * @return The moment object associated with the specified time
	 */
	public TankDriveMoment get(double t) {
		//Do binary search to find the closest approximation
		int start = 0;
		int end = moments.length - 1;
		int mid;
		
		//If t is greater than the entire length in time of the left side, return the last BasicMoment
		if(t >= moments[moments.length - 1].getTime())
			return moments[moments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			double midTime = moments[mid].getTime();
			
			if(midTime == t)
				return moments[mid].clone();
			//If we reached the end, return the end
			if(mid == moments.length - 1)
				return moments[mid].clone();
			
			double nextTime = moments[mid + 1].getTime();
			
			//If t is sandwiched between 2 existing times, the return the closest one
			if(midTime <= t && nextTime >= t) {
				double f = (t - midTime) / (nextTime - midTime);
				return new TankDriveMoment(MathUtils.lerp(moments[mid].getLeftPosition(), moments[mid + 1].getLeftPosition(), f), 
						MathUtils.lerp(moments[mid].getRightPosition(), moments[mid + 1].getRightPosition(), f),
						MathUtils.lerp(moments[mid].getLeftVelocity(), moments[mid + 1].getLeftVelocity(), f), 
						MathUtils.lerp(moments[mid].getRightVelocity(), moments[mid + 1].getRightVelocity(), f),
						MathUtils.lerp(moments[mid].getLeftAcceleration(), moments[mid + 1].getLeftAcceleration(), f), 
						MathUtils.lerp(moments[mid].getRightAcceleration(), moments[mid + 1].getRightAcceleration(), f),
						MathUtils.lerpAngle(headingVectors[mid], headingVectors[mid + 1], f), t, initialFacing);
			}
			//Continue the binary search if not found
			if(midTime < t) {
				start = mid;
				continue;
			}
			else if(midTime > t) {
				end = mid;
				continue;
			}
		}
	}
	
	/**
	 * Returns a modified trajectory in which every left turn becomes a right turn. Note that is operation is not 
	 * the same as reflecting across the Y axis, unless the first waypoint has a heading of pi/2.
	 * @return The mirrored trajectory
	 */
	public TankDriveTrajectory mirrorLeftRight() {
		//Create new path
		Path newPath = path.mirrorLeftRight();
		
		double refAngle = params.waypoints[0].getHeading();
		
		TankDriveMoment[] newMoments = new TankDriveMoment[moments.length];
		for(int i = 0; i < newMoments.length; i ++) {
			//Moment data stays the same, but left and right sides are swapped
			newMoments[i] = TankDriveMoment.fromComponents(moments[i].rightComponent(), moments[i].leftComponent());
			//See BasicTrajectory.mirrorLeftRight()
			newMoments[i].setHeading(MathUtils.mirrorAngle(moments[i].getHeading(), refAngle));
			newMoments[i].setInitialFacing(newMoments[0].getFacingAbsolute());
		}
		
		TrajectoryParams newParams = params.clone();
		newParams.waypoints = newPath.getWaypoints();
		return new TankDriveTrajectory(newMoments, newPath, specs, newParams);
	}
	/**
	 * Returns a modified trajectory, in which every forward movement becomes a backward movement. Note that this 
	 * operation is not the same as reflecting across the X axis, unless the first waypoint has a heading of
	 * pi/2.
	 * @return The mirrored trajectory
	 */
	public TankDriveTrajectory mirrorFrontBack() {
		//See BasicTrajectory.mirrorFrontBack()
		TankDriveMoment[] newMoments = new TankDriveMoment[moments.length];
		Path newPath = path.mirrorFrontBack();
		double refAngle = params.waypoints[0].getHeading() + Math.PI / 2;
		
		for(int i = 0; i < newMoments.length; i ++) {
			newMoments[i] = new TankDriveMoment(-moments[i].getLeftPosition(), -moments[i].getRightPosition(),
					-moments[i].getLeftVelocity(), -moments[i].getRightVelocity(), -moments[i].getLeftAcceleration(),
					-moments[i].getRightAcceleration(), MathUtils.mirrorAngle(moments[i].getHeading(), refAngle), moments[i].getTime());
		}
		for(int i = 0; i < newMoments.length; i ++) {
			newMoments[i].setInitialFacing(newMoments[0].getFacingAbsolute());
		}
		
		TrajectoryParams newParams = params.clone();
		newParams.waypoints = newPath.getWaypoints();
		return new TankDriveTrajectory(newMoments, newPath, specs, newParams);
	}
	/**
	 * Returns a trajectory that, when driven, will retrace the steps of this trajectory exactly.
	 * @return The retraced trajectory
	 */
	public TankDriveTrajectory retrace() {
		//See BasicTrajectory.retrace()
		TankDriveMoment[] newMoments = new TankDriveMoment[moments.length];
		
		
		//Create new path
		Path newPath = path.retrace();
		
		TankDriveMoment lastMoment = moments[moments.length - 1];
		
		for(int i = 0; i < newMoments.length; i ++) {
			TankDriveMoment current = moments[moments.length - 1 - i];
			
			/*
			 * To generate the new moments, first the order of the moments has to be reversed, since we
			 * are now starting from the end. The first moments should have less distance than the later moments,
			 * so when iterating backwards, the position of the moment is subtracted from the total distance,
			 * then negated since we're driving backwards. Velocity is also negated, but since it's not accumulative,
			 * it does not need to be subtracted from the total. Finally, acceleration is negated once for driving
			 * backwards, and negated again because the direction of time is reversed, and together they cancel
			 * out, resulting in no change. The heading is flipped 180 degrees, and the time is subtracted
			 * from the total.
			 */
			newMoments[i] = new TankDriveMoment(-(lastMoment.getLeftPosition() - current.getLeftPosition()),
					-(lastMoment.getRightPosition() - current.getRightPosition()), -current.getLeftVelocity(),
					-current.getRightVelocity(), current.getLeftAcceleration(), current.getRightAcceleration(),
					(current.getHeading() + Math.PI) % (2 * Math.PI), lastMoment.getTime() - current.getTime());
		}
		for(int i = 0; i < newMoments.length; i ++) {
			newMoments[i].setInitialFacing(newMoments[0].getFacingAbsolute());
		}

		TrajectoryParams newParams = params.clone();
		newParams.waypoints = newPath.getWaypoints();
		return new TankDriveTrajectory(newMoments, newPath, specs, newParams);
	}
}
