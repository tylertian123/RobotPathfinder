package robot.pathfinder.tankdrive;

import robot.pathfinder.core.BasicTrajectory;
import robot.pathfinder.core.BezierPath;
import robot.pathfinder.core.Moment;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;

public class TankDriveTrajectory {
	
	public static final class MomentKey {
		private MomentKey() {}
	}
	private static MomentKey momentKey = new MomentKey();
	
	//The internal path that this trajectory is based on
	BezierPath path;
	//"Moments" are generated for left and right separately
	Moment[] leftMoments, rightMoments;
	
	public TankDriveTrajectory(BasicTrajectory traj) {
		if(!traj.isTank()) {
			throw new IllegalArgumentException("Base trajectory is not generated with tank drive");
		}
		
		Moment[] moments = traj.getMoments();
		leftMoments = new Moment[moments.length];
		rightMoments = new Moment[moments.length];
		leftMoments[0] = new Moment(0, 0, 0, 0);
		rightMoments[0] = new Moment(0, 0, 0, 0);
		
		
		path = traj.getPath();
		path.setBaseRadius(traj.getRobotSpecs().getBaseWidth() / 2);
		
		Vec2D[] initPos = path.wheelsAt(0);
		Vec2D prevLeft = initPos[0], prevRight = initPos[1];
		
		for(int i = 1; i < moments.length; i ++) {
			Vec2D[] wheelPos = path.wheelsAt(moments[i].zz_getPathT(momentKey));
			double dxLeft = prevLeft.distTo(wheelPos[0]);
			double dxRight = prevRight.distTo(wheelPos[1]);
			double dt = moments[i].getTime() - moments[i - 1].getTime();
			
			prevLeft = wheelPos[0];
			prevRight = wheelPos[1];
			
			leftMoments[i] = new Moment();
			rightMoments[i] = new Moment();
			leftMoments[i].setPosition(leftMoments[i - 1].getPosition() + dxLeft);
			rightMoments[i].setPosition(rightMoments[i - 1].getPosition() + dxRight);
			leftMoments[i].setVelocity(moments[i].getVelocity() - moments[i].getVelocity() / moments[i].getR() * traj.getRobotSpecs().getBaseWidth() / 2);
			rightMoments[i].setVelocity(moments[i].getVelocity() + moments[i].getVelocity() / moments[i].getR() * traj.getRobotSpecs().getBaseWidth() / 2);
			leftMoments[i - 1].setAcceleration((leftMoments[i].getVelocity() - leftMoments[i - 1].getVelocity()) / dt);
			rightMoments[i - 1].setAcceleration((rightMoments[i].getVelocity() - rightMoments[i - 1].getVelocity()) / dt);
			leftMoments[i].setTime(moments[i].getTime());
			rightMoments[i].setTime(moments[i].getTime());
		}
	}
	
	protected TankDriveTrajectory(Moment[] lMoments, Moment[] rMoments, BezierPath path) {
		leftMoments = lMoments;
		rightMoments = rMoments;
		this.path = path;
	}
	
	/**
	 * Retrieves the {@code Moment} object associated with the left side at the specified time.<br>
	 * <br>
	 * This method retrieves the 2 {@code Moment} objects closest to the specified time, and lerps them to get an
	 * estimate.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the left side at time t
	 */
	public Moment getLeft(double t) {
		//Do binary search to find the closest approximation
		int start = 0;
		int end = leftMoments.length - 1;
		int mid;
		
		//If t is greater than the entire length in time of the left side, return the last Moment
		if(t >= leftMoments[leftMoments.length - 1].getTime())
			return leftMoments[leftMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			double midTime = leftMoments[mid].getTime();
			
			if(midTime == t)
				return leftMoments[mid];
			//If we reached the end, return the end
			if(mid == leftMoments.length - 1)
				return leftMoments[mid];
			
			double nextTime = leftMoments[mid + 1].getTime();
			
			//If t is sandwiched between 2 existing times, the return the closest one
			if(midTime <= t && nextTime >= t) {
				double f = (t - midTime) / (nextTime - midTime);
				return new Moment(MathUtils.lerp(leftMoments[mid].getPosition(), leftMoments[mid + 1].getPosition(), f),
						MathUtils.lerp(leftMoments[mid].getVelocity(), leftMoments[mid + 1].getVelocity(), f),
						MathUtils.lerp(leftMoments[mid].getAcceleration(), leftMoments[mid + 1].getAcceleration(), f));
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
	 * Retrieves the {@code Moment} object associated with the right side at the specified time.<br>
	 * <br>
	 * This method retrieves the 2 {@code Moment} objects closest to the specified time, and lerps them to get an
	 * estimate.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the right side at time t
	 */
	public Moment getRight(double t) {
		//Do binary search to find the closest approximation
		int start = 0;
		int end = rightMoments.length - 1;
		int mid;
		
		//If t is greater than the entire length in time of the left side, return the last Moment
		if(t >= rightMoments[rightMoments.length - 1].getTime())
			return rightMoments[rightMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			double midTime = rightMoments[mid].getTime();
			
			if(midTime == t)
				return rightMoments[mid];
			//If we reached the end, return the end
			if(mid == rightMoments.length - 1)
				return rightMoments[mid];
			
			double nextTime = rightMoments[mid + 1].getTime();
			
			//If t is sandwiched between 2 existing times, the return the closest one
			if(midTime <= t && nextTime >= t) {
				double f = (t - midTime) / (nextTime - midTime);
				return new Moment(MathUtils.lerp(rightMoments[mid].getPosition(), rightMoments[mid + 1].getPosition(), f),
						MathUtils.lerp(rightMoments[mid].getVelocity(), rightMoments[mid + 1].getVelocity(), f),
						MathUtils.lerp(rightMoments[mid].getAcceleration(), rightMoments[mid + 1].getAcceleration(), f));
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
	 * Retrieves the total time needed for the robot to complete this trajectory.
	 * @return The total time required to complete this trajectory
	 */
	public double totalTime() {
		//Return the length in time of the longer side
		//In reality they should be the same
		return Math.max(leftMoments[leftMoments.length - 1].getTime(), rightMoments[rightMoments.length - 1].getTime());
	}
	
	/**
	 * Retrieves the underlying {@code BezierPath} used to generate this trajectory.
	 * @return The internal {@code BezierPath}
	 */
	public BezierPath getPath() {
		return path;
	}
	/**
	 * Retrieves the array of {@code Moment} objects generated.
	 * @return An array of 2 arrays. The first contains the left side's {@code Moment}s, and the second
	 * contains the right side's {@code Moment}s
	 */
	public Moment[][] getMoments() {
		return new Moment[][] { leftMoments, rightMoments };
	}
	/**
	 * Accesses the internal {@code BezierPath} object and returns the path value at the specified time.<br>
	 * This value should <b>not</b> be used directly for motion planning. It is a path, not a trajectory.
	 * @param t A positive real number ranging from 0 to 1
	 * @return The X and Y values at the specified time on the path
	 */
	public Vec2D pathAt(double t) {
		return path.at(t);
	}
	
	/**
	 * Returns the left-right mirror image of this trajectory. Every left turn will now become a right turn.<br>
	 * <br>
	 * Internally, this is done by creating a new trajectory with the left and right wheels swapped. No new 
	 * arrays are created, so this method is very fast.
	 * @see TankDriveTrajectory#mirrorFrontBack() mirrorFrontBack()
	 * @return The mirrored trajectory
	 */
	public TankDriveTrajectory mirrorLeftRight() {
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Vec2D refPoint = new Vec2D(old[0]);
		Waypoint[] waypoints = new Waypoint[old.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			//Negate the relative x coordinates and flip the angles
			waypoints[i] = new Waypoint(-refPoint.relative(old[i].asVector()).getX(), old[i].getY(), -old[i].getHeading() + Math.PI);
		}
		
		//Just create a new one with the sides swapped
		return new TankDriveTrajectory(rightMoments, leftMoments, new BezierPath(waypoints, path.getAlpha(), path.getBaseRaidus()));
	}
	/**
	 * Returns the front-back mirror image of this trajectory. Every forwards movement will now become
	 * a backwards movement.<br>
	 * <b>Warning: The new trajectory created does not respect maximum deceleration constraints. If you wish
	 * for the new trajectory to respect maximum deceleration constrains, construct a new trajectory with
	 * maximum acceleration and deceleration swapped, and then call this method.</b><br>
	 * <br>
	 * Internally, this is done by creating a new trajectory in which every {@code Moment}'s position, velocity
	 * and acceleration are negated. This means new arrays are being created and copied and thus this method
	 * could be slow for trajectories with many segments.
	 * 
	 * @see TankDriveTrajectory#mirrorLeftRight() mirrorLeftRight()
	 * @return The mirrored trajectory
	 */
	public TankDriveTrajectory mirrorFrontBack() {
		Moment[] lMoments = new Moment[leftMoments.length];
		Moment[] rMoments = new Moment[rightMoments.length];
		
		for(int i = 0; i < lMoments.length; i ++) {
			//Negate the distances, velocities and accelerations to drive backwards
			//Time, of course, always stays positive.
			lMoments[i] = new Moment(-leftMoments[i].getPosition(), -leftMoments[i].getVelocity(), -leftMoments[i].getAcceleration(), leftMoments[i].getTime());
			rMoments[i] = new Moment(-rightMoments[i].getPosition(), -rightMoments[i].getVelocity(), -rightMoments[i].getAcceleration(), rightMoments[i].getTime());
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Vec2D refPoint = new Vec2D(old[0]);
		Waypoint[] waypoints = new Waypoint[old.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			//Negate the relative y coordinates and the headings and keep the x coordinates
			waypoints[i] = new Waypoint(old[i].getX(), -refPoint.relative(old[i].asVector()).getY(), -old[i].getHeading());
		}
		
		BezierPath newPath = new BezierPath(waypoints, path.getAlpha(), path.getBaseRaidus());
		newPath.setDrivingBackwards(true);
		
		return new TankDriveTrajectory(lMoments, rMoments, newPath);
	}
	/**
	 * Returns the reverse of this trajectory. Not to be confused with {@link #retrace()}.
	 * The new trajectory starts at the end of this trajectory and ends at the beginning of it. 
	 * However because the direction is not reversed, the new trajectory does not retrace this trajectory.
	 * Use {@link #retrace()} instead.<br>
	 * <b>Warning: The new trajectory created does not respect maximum deceleration constraints. If you wish
	 * for the new trajectory to respect maximum deceleration constrains, construct a new trajectory with
	 * maximum acceleration and deceleration swapped, and then call this method.</b><br>
	 * <br>
	 * Internally, this is done by creating a new trajectory in which the movements are reversed, that is, the
	 * last {@code Moment}s become the first {@code Moment}s, with the distances adjusted. This means new arrays are being created and copied and thus this method
	 * could be slow for trajectories with many segments.
	 * @see #retrace()
	 * @return The reverse of this trajectory
	 */
	public TankDriveTrajectory reverse() {
		Moment[] lMoments = new Moment[leftMoments.length];
		Moment[] rMoments = new Moment[rightMoments.length];
		
		Moment lLast = leftMoments[leftMoments.length - 1];
		Moment rLast = rightMoments[rightMoments.length - 1];
		
		for(int i = 0; i < lMoments.length; i ++) {
			Moment lm = leftMoments[leftMoments.length - 1 - i];
			Moment rm = rightMoments[rightMoments.length - 1 - i];
			
			//The velocities stay the same
			//Distance and time have to be adjusted since the last Moment has to have distance 0 and time 0
			//Accelerations are negated because if time is reversed, acceleration becomes deceleration
			lMoments[i] = new Moment(lLast.getPosition() - lm.getPosition(), lm.getVelocity(), -lm.getAcceleration(), lLast.getTime() - lm.getTime());
			rMoments[i] = new Moment(rLast.getPosition() - rm.getPosition(), rm.getVelocity(), -rm.getAcceleration(), rLast.getTime() - rm.getTime());
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Waypoint[] waypoints = new Waypoint[old.length];
		for(int i = 0; i < old.length; i ++) {
			//New path is just the same as the old path, but with the order of the waypoints reversed,
			//and headings flipped
			waypoints[old.length - 1 - i] = new Waypoint(old[i].getX(), old[i].getY(), (old[i].getHeading() + Math.PI) % (2 * Math.PI));
		}
		return new TankDriveTrajectory(lMoments, rMoments, new BezierPath(waypoints, path.getAlpha(), path.getBaseRaidus()));
	}
	/**
	 * Returns the trajectory that, when driven, would retrace this trajectory and return the robot to its
	 * original position. Not to be confused with {@link #reverse()}.<br>
	 * <b>Warning: The new trajectory created does not respect maximum deceleration constraints. If you wish
	 * for the new trajectory to respect maximum deceleration constrains, construct a new trajectory with
	 * maximum acceleration and deceleration swapped, and then call this method.</b><br>
	 * <br>
	 * Calling this method is essentially equivalent to calling {@link #reverse()} and {@code #mirrorFrontBack()}
	 * in order, but with optimization. 
	 * @see #reverse()
	 * @return The trajectory that retraces this one
	 */
	public TankDriveTrajectory retrace() {
		Moment[] lMoments = new Moment[leftMoments.length];
		Moment[] rMoments = new Moment[rightMoments.length];
		
		Moment lLast = leftMoments[leftMoments.length - 1];
		Moment rLast = rightMoments[rightMoments.length - 1];
		
		for(int i = 0; i < lMoments.length; i ++) {
			Moment lm = leftMoments[leftMoments.length - 1 - i];
			Moment rm = rightMoments[rightMoments.length - 1 - i];

			//A combination of reverse() and mirrorFrontBack()
			lMoments[i] = new Moment(-(lLast.getPosition() - lm.getPosition()), -lm.getVelocity(), lm.getAcceleration(), lLast.getTime() - lm.getTime());
			rMoments[i] = new Moment(-(rLast.getPosition() - rm.getPosition()), -rm.getVelocity(), rm.getAcceleration(), rLast.getTime() - rm.getTime());
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Waypoint[] waypoints = new Waypoint[old.length];
		for(int i = 0; i < old.length; i ++) {
			//New path is just the same as the old path, but with the order of the waypoints reversed,
			//and headings flipped
			waypoints[old.length - 1 - i] = new Waypoint(old[i].getX(), old[i].getY(), (old[i].getHeading() + Math.PI) % (2 * Math.PI));
		}
		BezierPath newPath = new BezierPath(waypoints, path.getAlpha(), path.getBaseRaidus());
		newPath.setDrivingBackwards(true);
	
		//Note that even though the final path looks exactly the same, the order of the waypoints is actually
		//the opposite.
		return new TankDriveTrajectory(lMoments, rMoments, newPath);
	}
}
