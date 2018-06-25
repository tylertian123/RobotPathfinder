package robot.pathfinder.core;

import java.util.ArrayList;

import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.util.Pair;

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
		
		ArrayList<Pair<Double, Double>> points = new ArrayList<>(segmentCount);
		
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
				
				points.add(new Pair<>(t, vMax));
			}
		}
		else {
			for(int i = 0; i < segmentCount; i ++) {
				points.add(new Pair<>(t_delta * i, maxVelocity));
			}
		}
		
		moments = new Moment[segmentCount];
		moments[0] = new Moment(0, 0, 0, 0);
		
		double totalDist = path.computePathLength(segmentCount * 50);
		double distPerIteration = totalDist / (segmentCount - 1);
		
		if(isTank) {
			moments[0].zz_setPathT(0, momentKey);
		}
		for(int i = 1; i < moments.length; i ++) {
			double accumulatedDist = i * distPerIteration;
			
			double theoreticalMax = points.get(i).getElem2();
			
			if(moments[i - 1].getVelocity() < theoreticalMax) {
				double distDiff = distPerIteration;
				
				double maxVel = Math.sqrt(Math.pow(moments[i - 1].getVelocity(), 2) + 2 * maxAcceleration * distDiff);
				
				double vel;
				if(maxVel > theoreticalMax) {
					double accel = (Math.pow(theoreticalMax, 2) - Math.pow(moments[i - 1].getVelocity(), 2)) / (2 * distDiff);
					vel = theoreticalMax;
					moments[i - 1].setAcceleration(accel);
				}
				else {
					vel = maxVel;
					moments[i - 1].setAcceleration(maxAcceleration);
				}
				
				moments[i] = new Moment(accumulatedDist, vel, 0);
			}
			else {
				moments[i] = new Moment(accumulatedDist, theoreticalMax, 0);
				moments[i - 1].setAcceleration(0);
			}

			if(isTank) {
				moments[i].zz_setPathT(path.s2T(points.get(i).getElem1()), momentKey);
			}
		}
		
		moments[moments.length - 1].setVelocity(0);
		moments[moments.length - 1].setAcceleration(0);
		
		for(int i = moments.length - 2; i >= 0; i --) {
			
			if(moments[i].getVelocity() > moments[i + 1].getVelocity()) {
			
				double distDiff = moments[i + 1].getPosition() - moments[i].getPosition();
				
				double maxVel = Math.sqrt(Math.pow(moments[i + 1].getVelocity(), 2) + 2 * maxAcceleration * distDiff);
				
				double vel;
				if(maxVel > moments[i].getVelocity()) {
					double accel = (Math.pow(moments[i].getVelocity(), 2) - Math.pow(moments[i + 1].getVelocity(), 2)) / (2 * distDiff);
					moments[i].setAcceleration(-accel);
					vel = moments[i].getVelocity();
				}
				else {
					vel = maxVel;
					moments[i].setAcceleration(-maxAcceleration);
				}
				
				moments[i].setVelocity(vel);
			}
		}
		
		for(int i = 1; i < moments.length; i ++) {
			
			double dt = MathUtils.findPositiveQuadraticRoot(moments[i - 1].getAcceleration() / 2, moments[i - 1].getVelocity(), 
					-(moments[i].getPosition() - moments[i - 1].getPosition()), minUnit);
			moments[i].setTime(moments[i - 1].getTime() + dt);
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
			
			if(midTime <= t && nextTime >= t) {
				
				double f = (t - midTime) / (nextTime - midTime);
				return new Moment(MathUtils.lerp(moments[mid].getPosition(), moments[mid + 1].getPosition(), f),
						MathUtils.lerp(moments[mid].getVelocity(), moments[mid + 1].getVelocity(), f),
						MathUtils.lerp(moments[mid].getAcceleration(), moments[mid + 1].getAcceleration(), f));
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


