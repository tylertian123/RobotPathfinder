package robot.pathfinder.core.trajectory;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.Path;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;

public class TankDriveTrajectory {
	
	//The internal path that this trajectory is based on
	Path path;
	//"Moments" are generated for left and right separately
	TankDriveMoment[] moments;
	Vec2D[] headingVectors;
	
	RobotSpecs specs;
	TrajectoryParams params;
	
	public TankDriveTrajectory(BasicTrajectory traj) {
		if(!traj.isTank()) {
			throw new IllegalArgumentException("Base trajectory is not generated with tank drive");
		}
		
		BasicMoment[] trajMoments = traj.getMoments();
		moments = new TankDriveMoment[trajMoments.length];
		moments[0] = new TankDriveMoment(0, 0, 0, 0, 0, 0, trajMoments[0].getHeading(), 0);
		
		specs = traj.getRobotSpecs();
		params = traj.getGenerationParams();
		headingVectors = traj.headingVectors;
		
		path = traj.getPath();
		RobotSpecs specs = traj.getRobotSpecs();
		double baseRadius = specs.getBaseWidth() / 2;
		double maxVel = specs.getMaxVelocity();
		double maxAccel = specs.getMaxAcceleration();
		path.setBaseRadius(traj.getRobotSpecs().getBaseWidth() / 2);
		
		Vec2D[] initPos = path.wheelsAt(0);
		Vec2D prevLeft = initPos[0], prevRight = initPos[1];
		
		for(int i = 1; i < trajMoments.length; i ++) {
			Vec2D[] wheelPos = path.wheelsAt(traj.pathT[i]);
			double dxLeft = prevLeft.distTo(wheelPos[0]);
			double dxRight = prevRight.distTo(wheelPos[1]);
			double dt = trajMoments[i].getTime() - trajMoments[i - 1].getTime();
			
			prevLeft = wheelPos[0];
			prevRight = wheelPos[1];
			
			moments[i] = new TankDriveMoment();
			moments[i].setLeftPosition(moments[i - 1].getLeftPosition() + dxLeft);
			moments[i].setRightPosition(moments[i - 1].getRightPosition() + dxRight);
			double vel = trajMoments[i].getVelocity();
			moments[i].setLeftVelocity(MathUtils.clampAbs(vel - vel / traj.pathRadius[i] * baseRadius, maxVel));
			moments[i].setRightVelocity(MathUtils.clampAbs(vel + vel / traj.pathRadius[i] * baseRadius, maxVel));
			if(moments[i].getLeftVelocity() < 0 || moments[i].getRightVelocity() < 0) {
				throw new TrajectoryGenerationException("Error: Negative distance functionality is not implemented so the trajectory is impossible");
			}
			moments[i - 1].setLeftAcceleration(MathUtils.clampAbs((moments[i].getLeftVelocity() - moments[i - 1].getLeftVelocity()) / dt, maxAccel));
			moments[i - 1].setRightAcceleration(MathUtils.clampAbs((moments[i].getRightVelocity() - moments[i - 1].getRightVelocity()) / dt, maxAccel));
			moments[i].setTime(trajMoments[i].getTime());
			moments[i].setHeading(trajMoments[i].getHeading());
		}
	}
	
	protected TankDriveTrajectory(TankDriveMoment[] moments, Path path) {
		this.moments = moments;
		this.path = path;
		
		headingVectors = new Vec2D[moments.length];
		for(int i = 0; i < moments.length; i ++) {
			headingVectors[i] = new Vec2D(Math.cos(moments[i].getHeading()), Math.sin(moments[i].getHeading()));
		}
	}
	
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
				return moments[mid];
			//If we reached the end, return the end
			if(mid == moments.length - 1)
				return moments[mid];
			
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
						MathUtils.lerpAngle(headingVectors[mid], headingVectors[mid + 1], f),
						MathUtils.lerp(moments[mid].getTime(), moments[mid + 1].getTime(), f));
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
		return moments[moments.length - 1].getTime();
	}
	
	/**
	 * Retrieves the underlying {@code BezierPath} used to generate this trajectory.
	 * @return The internal {@code BezierPath}
	 */
	public Path getPath() {
		return path;
	}
	
	public TankDriveMoment[] getMoments() {
		return moments;
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
		Path path = Path.constructPath(params.pathType, waypoints, params.alpha);
		path.setBaseRadius(specs.getBaseWidth() / 2);
		
		TankDriveMoment[] newMoments = new TankDriveMoment[moments.length];
		for(int i = 0; i < newMoments.length; i ++) {
			newMoments[i] = TankDriveMoment.fromComponents(moments[i].rightComponent(), moments[i].leftComponent());
			newMoments[i].setHeading(-moments[i].getHeading() + Math.PI);
		}
		
		//Just create a new one with the sides swapped
		return new TankDriveTrajectory(newMoments, path);
	}
	/**
	 * Returns the front-back mirror image of this trajectory. Every forwards movement will now become
	 * a backwards movement.<br>
	 * <b>Warning: The new trajectory created does not respect maximum deceleration constraints. If you wish
	 * for the new trajectory to respect maximum deceleration constrains, construct a new trajectory with
	 * maximum acceleration and deceleration swapped, and then call this method.</b><br>
	 * <br>
	 * Internally, this is done by creating a new trajectory in which every {@code BasicMoment}'s position, velocity
	 * and acceleration are negated. This means new arrays are being created and copied and thus this method
	 * could be slow for trajectories with many segments.
	 * 
	 * @see TankDriveTrajectory#mirrorLeftRight() mirrorLeftRight()
	 * @return The mirrored trajectory
	 *//*
	public TankDriveTrajectory mirrorFrontBack() {
		BasicMoment[] lMoments = new BasicMoment[leftMoments.length];
		BasicMoment[] rMoments = new BasicMoment[rightMoments.length];
		
		for(int i = 0; i < lMoments.length; i ++) {
			//Negate the distances, velocities and accelerations to drive backwards
			//Time, of course, always stays positive.
			lMoments[i] = new BasicMoment(-leftMoments[i].getPosition(), -leftMoments[i].getVelocity(), -leftMoments[i].getAcceleration(), leftMoments[i].getTime());
			rMoments[i] = new BasicMoment(-rightMoments[i].getPosition(), -rightMoments[i].getVelocity(), -rightMoments[i].getAcceleration(), rightMoments[i].getTime());
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Vec2D refPoint = new Vec2D(old[0]);
		Waypoint[] waypoints = new Waypoint[old.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			//Negate the relative y coordinates and the headings and keep the x coordinates
			waypoints[i] = new Waypoint(old[i].getX(), -refPoint.relative(old[i].asVector()).getY(), -old[i].getHeading());
		}
		
		BezierPath newPath = new BezierPath(waypoints, params.alpha, path.getBaseRaidus());
		newPath.setDrivingBackwards(true);
		
		return new TankDriveTrajectory(lMoments, rMoments, newPath);
	}
	*//**
	 * Returns the reverse of this trajectory. Not to be confused with {@link #retrace()}.
	 * The new trajectory starts at the end of this trajectory and ends at the beginning of it. 
	 * However because the direction is not reversed, the new trajectory does not retrace this trajectory.
	 * Use {@link #retrace()} instead.<br>
	 * <b>Warning: The new trajectory created does not respect maximum deceleration constraints. If you wish
	 * for the new trajectory to respect maximum deceleration constrains, construct a new trajectory with
	 * maximum acceleration and deceleration swapped, and then call this method.</b><br>
	 * <br>
	 * Internally, this is done by creating a new trajectory in which the movements are reversed, that is, the
	 * last {@code BasicMoment}s become the first {@code BasicMoment}s, with the distances adjusted. This means new arrays are being created and copied and thus this method
	 * could be slow for trajectories with many segments.
	 * @see #retrace()
	 * @return The reverse of this trajectory
	 *//*
	public TankDriveTrajectory reverse() {
		BasicMoment[] lMoments = new BasicMoment[leftMoments.length];
		BasicMoment[] rMoments = new BasicMoment[rightMoments.length];
		
		BasicMoment lLast = leftMoments[leftMoments.length - 1];
		BasicMoment rLast = rightMoments[rightMoments.length - 1];
		
		for(int i = 0; i < lMoments.length; i ++) {
			BasicMoment lm = leftMoments[leftMoments.length - 1 - i];
			BasicMoment rm = rightMoments[rightMoments.length - 1 - i];
			
			//The velocities stay the same
			//Distance and time have to be adjusted since the last BasicMoment has to have distance 0 and time 0
			//Accelerations are negated because if time is reversed, acceleration becomes deceleration
			lMoments[i] = new BasicMoment(lLast.getPosition() - lm.getPosition(), lm.getVelocity(), -lm.getAcceleration(), lLast.getTime() - lm.getTime());
			rMoments[i] = new BasicMoment(rLast.getPosition() - rm.getPosition(), rm.getVelocity(), -rm.getAcceleration(), rLast.getTime() - rm.getTime());
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Waypoint[] waypoints = new Waypoint[old.length];
		for(int i = 0; i < old.length; i ++) {
			//New path is just the same as the old path, but with the order of the waypoints reversed,
			//and headings flipped
			waypoints[old.length - 1 - i] = new Waypoint(old[i].getX(), old[i].getY(), (old[i].getHeading() + Math.PI) % (2 * Math.PI));
		}
		return new TankDriveTrajectory(lMoments, rMoments, new BezierPath(waypoints, params.alpha, path.getBaseRaidus()));
	}
	*//**
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
	 *//*
	public TankDriveTrajectory retrace() {
		BasicMoment[] lMoments = new BasicMoment[leftMoments.length];
		BasicMoment[] rMoments = new BasicMoment[rightMoments.length];
		
		BasicMoment lLast = leftMoments[leftMoments.length - 1];
		BasicMoment rLast = rightMoments[rightMoments.length - 1];
		
		for(int i = 0; i < lMoments.length; i ++) {
			BasicMoment lm = leftMoments[leftMoments.length - 1 - i];
			BasicMoment rm = rightMoments[rightMoments.length - 1 - i];

			//A combination of reverse() and mirrorFrontBack()
			lMoments[i] = new BasicMoment(-(lLast.getPosition() - lm.getPosition()), -lm.getVelocity(), lm.getAcceleration(), lLast.getTime() - lm.getTime());
			rMoments[i] = new BasicMoment(-(rLast.getPosition() - rm.getPosition()), -rm.getVelocity(), rm.getAcceleration(), rLast.getTime() - rm.getTime());
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Waypoint[] waypoints = new Waypoint[old.length];
		for(int i = 0; i < old.length; i ++) {
			//New path is just the same as the old path, but with the order of the waypoints reversed,
			//and headings flipped
			waypoints[old.length - 1 - i] = new Waypoint(old[i].getX(), old[i].getY(), (old[i].getHeading() + Math.PI) % (2 * Math.PI));
		}
		BezierPath newPath = new BezierPath(waypoints, params.alpha, path.getBaseRaidus());
		newPath.setDrivingBackwards(true);
	
		//Note that even though the final path looks exactly the same, the order of the waypoints is actually
		//the opposite.
		return new TankDriveTrajectory(lMoments, rMoments, newPath);
	}*/
	
	public RobotSpecs getRobotSpecs() {
		return specs;
	}
	public TrajectoryParams getGenerationParams() {
		return params;
	}
}
