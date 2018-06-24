package robot.pathfinder.core.trajectory;

import robot.pathfinder.core.Moment;
import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.BezierPath;
import robot.pathfinder.core.path.PathPoint;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;

public class BasicTrajectory {
	
	public static final class MomentKey {
		private MomentKey() {}
	}
	private static MomentKey momentKey = new MomentKey();
	
	BezierPath path;
	
	Moment[] moments;
	
	RobotSpecs robotSpecs;
	TrajectoryParams params;
	
	boolean isTank;
	
	static double minUnit = 1.0e-6;
	
	static void setSolverRoundingLimit(double limit) {
		minUnit = limit;
	}
	static double getSolverRoundingLimit() {
		return minUnit;
	}
	
	public BasicTrajectory(RobotSpecs specs, TrajectoryParams params) {
		this.params = params;
		this.robotSpecs = specs;
		
		boolean isTank = params.isTank;
		Waypoint[] waypoints = params.waypoints;
		int segmentCount = params.segmentCount;
		double alpha = params.alpha;
		double maxVelocity = specs.getMaxVelocity();
		double maxAcceleration = specs.getMaxAcceleration();
		double baseWidth = specs.getBaseWidth();
		
		if(waypoints == null) {
			throw new IllegalArgumentException("Waypoints cannot be null!");
		}
		if(Double.isNaN(alpha)) {
			throw new IllegalArgumentException("Alpha is not set, or is NaN");
		}
		
		this.isTank = isTank;
		path = new BezierPath(waypoints, alpha);
		double t_delta = 1.0 / segmentCount;
		
		PathPoint[] points = new PathPoint[segmentCount];
		
		if(isTank) {
			for(int i = 0; i < segmentCount; i ++) {
				double t = t_delta * i;
				
				Vec2D deriv = path.derivAt(t);
				double xDeriv = deriv.getX();
				double yDeriv = deriv.getY();
				Vec2D secondDeriv = path.secondDerivAt(t);
				double xSecondDeriv = secondDeriv.getX();
				double ySecondDeriv = secondDeriv.getY();
				double curvature = MathUtils.curvature(xDeriv, xSecondDeriv, yDeriv, ySecondDeriv);
				
				//R is always positive
				double r = Math.abs(1 / curvature);
				//Compute max speed of entire robot with a positive R
				double vMax = (2 * maxVelocity) / (2 + baseWidth / r);
				
				points[i] = new PathPoint(t, vMax);
			}
		}
		else {
			for(int i = 0; i < segmentCount; i ++) {
				points[i] = new PathPoint(t_delta * i, maxVelocity);
			}
		}
		
		moments = new Moment[segmentCount];
		moments[0] = new Moment(0, 0, 0, 0);
		path.resetIntegration();
		
		double prevT = 0;
		if(isTank) {
			moments[0].setPathT(0, momentKey);
		}
		for(int i = 1; i < moments.length; i ++) {
			double dt = points[i].getLocation() - points[i - 1].getLocation();
			
			double accumulatedDist = path.integrateLen(dt);
			double distDiff = accumulatedDist - moments[i - 1].getPosition();
			
			double maxVel = Math.sqrt(Math.pow(moments[i - 1].getVelocity(), 2) + 2 * maxAcceleration * distDiff);
			
			double vel;
			if(maxVel > points[i].getMaxVel()) {
				vel = points[i].getMaxVel();
				moments[i - 1].setAcceleration(0);
			}
			else {
				vel = maxVel;
				moments[i - 1].setAcceleration(maxAcceleration);
			}
			
			moments[i] = new Moment(accumulatedDist, vel, 0);

			if(isTank) {
				moments[i].setPathT(prevT += dt, momentKey);
			}
		}
		
		moments[moments.length - 1].setVelocity(0);
		moments[moments.length - 1].setAcceleration(0);
		
		for(int i = moments.length - 2; i >= 0; i --) {
			double distDiff = moments[i + 1].getPosition() - moments[i].getPosition();
			
			double maxVel = Math.sqrt(Math.pow(moments[i + 1].getVelocity(), 2) + 2 * maxAcceleration * distDiff);
			
			double vel;
			if(maxVel > moments[i].getVelocity()) {
				vel = moments[i].getVelocity();
			}
			else {
				vel = maxVel;
				moments[i + 1].setAcceleration(-maxAcceleration);
			}
			
			moments[i].setVelocity(vel);
		}
		
		for(int i = 1; i < moments.length; i ++) {
			
			double dt = MathUtils.findPositiveQuadraticRoot(moments[i - 1].getAcceleration() / 2, moments[i - 1].getVelocity(), 
					-(moments[i].getPosition() - moments[i - 1].getPosition()), minUnit);
			moments[i].setTime(moments[i - 1].getTime() + dt);
			moments[i].lock();
		}
	}
	
	public BezierPath getPath() {
		return path;
	}
	
	public Moment[] getMoments() {
		return moments;
	}
	
	public double totalTime() {
		return moments[moments.length - 1].getTime();
	}
	
	public Moment get(double t) {
		//Do binary search to find the closest approximation
		int start = 0;
		int end = moments.length - 1;
		int mid;
		
		//If t is greater than the entire length in time of the left side, return the last Moment
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
				//Get the slopes
				double dt = nextTime - midTime;
				double mAccel = (moments[mid + 1].getAcceleration() - moments[mid].getAcceleration()) / dt;
				double mVel = (moments[mid + 1].getVelocity() - moments[mid].getVelocity()) / dt;
				double mDist = (moments[mid + 1].getPosition() - moments[mid].getPosition()) / dt;
				//Linear approximation
				double t2 = t - midTime;
				return new Moment(mDist * t2 + moments[mid].getPosition(), 
						mVel * t2 + moments[mid].getVelocity(),
						mAccel * t2 + moments[mid].getAcceleration(), t);
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
	
	public boolean isTank() {
		return isTank;
	}
	
	public RobotSpecs getRobotSpecs() {
		return robotSpecs;
	}
	public TrajectoryParams getGenerationParams() {
		return params;
	}
}


