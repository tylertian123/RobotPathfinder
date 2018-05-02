package robot.pathfinder;

import robot.pathfinder.bezier.BezierPath;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.Moment;

public class TankDriveTrajectory {
	
	BezierPath path;
	TankDrivePathSegment[] segments;
	Moment[] leftMoments, rightMoments;
	double[] leftTimes, rightTimes;
	
	final double maxVel, maxAccel, baseWidth;
	
	public TankDriveTrajectory(Waypoint[] waypoints, double maxVelocity, double maxAcceleration, double baseWidth, double alpha, int segmentCount) {
		maxVel = maxVelocity;
		maxAccel = maxAcceleration;
		this.baseWidth = baseWidth;
		
		path = new BezierPath(waypoints, alpha);
		path.setBaseRadius(baseWidth / 2);
		double t_delta = 1.0 / segmentCount;
		double t_delta2 = t_delta / 2;
		
		segments = new TankDrivePathSegment[segmentCount];
		
		//Vec2D lastEnd = path.at(0);
		for(int i = 0; i < segmentCount; i ++) {
			double start = i * t_delta;
			double mid = start + t_delta2;
			double end = start + t_delta;
			segments[i] = new TankDrivePathSegment(start, end);
			
			//Vec2D endPointVector = path.at(end);
			//Vec2D midPointVector = path.at(mid);
			
			Vec2D deriv = path.derivAt(mid);
			double xDeriv = deriv.getX();
			double yDeriv = deriv.getY();
			Vec2D secondDeriv = path.secondDerivAt(mid);
			double xSecondDeriv = secondDeriv.getX();
			double ySecondDeriv = secondDeriv.getY();
			double curvature = (xDeriv * ySecondDeriv - yDeriv * xSecondDeriv) /
					Math.pow(xDeriv * xDeriv + yDeriv * yDeriv, 3.0/2.0);
			double r = Math.abs(1 / curvature);
			
			double vMax = (2 * maxVel) / (2 + baseWidth / r);
			double omega = Math.copySign(vMax / r, curvature);
			
			double vMaxRight = (2 * vMax + omega * baseWidth) / 2;
			double vMaxLeft = 2 * vMax - vMaxRight;
			
			segments[i].setMaxVelocities(vMaxLeft, vMaxRight);
			//lastEnd = endPointVector;
		}
		
		leftMoments = new Moment[segmentCount + 1];
		rightMoments = new Moment[segmentCount + 1];
		
		leftMoments[0] = new Moment(0, 0, 0);
		rightMoments[0] = new Moment(0, 0, 0);
		
		for(int i = 1; i < segmentCount + 1; i ++) {
			double dt = segments[i - 1].getEnd() - segments[i - 1].getStart();
			
			//Integrate the path of both sides separately
			double[] lastDists = path.getIntegratedWheelLens();
			double[] currentDists = path.integrateWheelLens(dt);
			double distDiffLeft = currentDists[0] - lastDists[0];
			double distDiffRight = currentDists[1] - lastDists[1];
			//System.out.printf("l: %f, r: %f\n", currentDists[0], currentDists[1]);
			//System.out.printf("l: %f, r: %f\n", distDiffLeft, distDiffRight);
			
			//Use the fourth kinematic equation to figure out the maximum reachable velocity for each side
			double leftMax = Math.sqrt(Math.pow(leftMoments[i - 1].getVelocity(), 2) + 2 * maxAccel * distDiffLeft);
			double rightMax = Math.sqrt(Math.pow(rightMoments[i - 1].getVelocity(), 2) + 2 * maxAccel * distDiffRight);
			double leftVel, rightVel;
			
			if(leftMax > segments[i - 1].getLeftMaxVelocity()) {
				leftVel = segments[i - 1].getLeftMaxVelocity();
				leftMoments[i - 1].setAcceleration(0);
			}
			else {
				leftVel = leftMax;
				leftMoments[i - 1].setAcceleration(maxAccel);
			}
			if(rightMax > segments[i - 1].getRightMaxVelocity()) {
				rightVel = segments[i - 1].getRightMaxVelocity();
				rightMoments[i - 1].setAcceleration(0);
			}
			else {
				rightVel = rightMax;
				rightMoments[i - 1].setAcceleration(maxAccel);
			}
			
			leftMoments[i] = new Moment(currentDists[0], leftVel, 0);
			rightMoments[i] = new Moment(currentDists[1], rightVel, 0);
		}
		
		leftMoments[segments.length].setVelocity(0);
		leftMoments[segments.length].setAcceleration(0);
		rightMoments[segments.length].setVelocity(0);
		rightMoments[segments.length].setAcceleration(0);
		for(int i = segments.length - 1; i >= 0; i --) {
			double leftDistDiff = leftMoments[i + 1].getDistance() - leftMoments[i].getDistance();
			double rightDistDiff = rightMoments[i + 1].getDistance() - rightMoments[i].getDistance();
			
			double leftMax = Math.sqrt(Math.pow(leftMoments[i + 1].getVelocity(), 2) + 2 * maxAccel * leftDistDiff);
			double rightMax = Math.sqrt(Math.pow(rightMoments[i + 1].getVelocity(), 2) + 2 * maxAccel * rightDistDiff);
			double leftVel, rightVel;
			
			if(leftMax > leftMoments[i].getVelocity()) {
				leftVel = leftMoments[i].getVelocity();
			}
			else {
				leftVel = leftMax;
				leftMoments[i].setAcceleration(-maxAccel);
			}
			leftMoments[i].setVelocity(leftVel);
			
			if(rightMax > rightMoments[i].getVelocity()) {
				rightVel = rightMoments[i].getVelocity();
			}
			else {
				rightVel = rightMax;
				rightMoments[i].setAcceleration(-maxAccel);
			}
			rightMoments[i].setVelocity(rightVel);
		}
		
		leftTimes = new double[leftMoments.length];
		rightTimes = new double[rightMoments.length];
		leftTimes[0] = 0;
		rightTimes[0] = 0;
		for(int i = 1; i < leftMoments.length; i ++) {
			double distDiffLeft = leftMoments[i].getDistance() - leftMoments[i - 1].getDistance();
			double distDiffRight = rightMoments[i].getDistance() - rightMoments[i - 1].getDistance();
			double leftVel = leftMoments[i - 1].getVelocity();
			double rightVel = rightMoments[i - 1].getVelocity();
			double leftAccel = leftMoments[i - 1].getAcceleration();
			double rightAccel = rightMoments[i - 1].getAcceleration();
			
			//Use the second kinematic formula to find the amount of time needed to reach the desired distance
			//This time is then used to give every Moment a timestamp
			double dtLeft = MathUtils.findPositiveQuadraticRoot(leftAccel / 2, leftVel, -distDiffLeft);
			double dtRight = MathUtils.findPositiveQuadraticRoot(rightAccel / 2, rightVel, -distDiffRight);
			if(Double.isNaN(dtLeft)) {
				System.out.printf("*LEFT* Accel: %f, Velo: %f, Dist: %f, Iteration: %d\n", leftAccel, leftVel, distDiffLeft, i);
			}
			if(Double.isNaN(dtRight)) {
				System.out.printf("*RIGHT* Accel: %f, Velo: %f, Dist: %f, Iteration: %d\n", rightAccel, rightVel, distDiffRight, i);
			}
			
			leftMoments[i].setTime(leftMoments[i - 1].getTime() + dtLeft);
			rightMoments[i].setTime(rightMoments[i - 1].getTime() + dtRight);
			leftTimes[i] = leftMoments[i].getTime();
			rightTimes[i] = rightMoments[i].getTime();
			//System.out.printf("l: %f, r: %f\n", leftTimes[i], rightTimes[i]);
		}
	}
	
	public Moment getLeft(double t) {
		int start = 0;
		int end = leftTimes.length - 1;
		int mid;
		
		if(t >= leftTimes[leftTimes.length - 1])
			return leftMoments[leftMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			if(leftTimes[mid] == t)
				return leftMoments[mid];
			
			if(mid == leftTimes.length - 1)
				return leftMoments[mid];
			
			if(leftTimes[mid] <= t && leftTimes[mid + 1] >= t) {
				if(Math.abs(t - leftTimes[mid]) > Math.abs(t - leftTimes[mid + 1])) {
					return leftMoments[mid + 1];
				}
				else {
					return leftMoments[mid];
				}
			}
			
			if(leftTimes[mid] < t) {
				start = mid;
				continue;
			}
			else if(leftTimes[mid] > t) {
				end = mid;
				continue;
			}
		}
	}
	public Moment getRight(double t) {
		int start = 0;
		int end = rightTimes.length - 1;
		int mid;
		
		if(t >= rightTimes[rightTimes.length - 1])
			return rightMoments[rightMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			if(rightTimes[mid] == t)
				return rightMoments[mid];
			
			if(mid == rightTimes.length - 1)
				return rightMoments[mid];
			
			if(rightTimes[mid] <= t && rightTimes[mid + 1] >= t) {
				if(Math.abs(t - rightTimes[mid]) > Math.abs(t - rightTimes[mid + 1])) {
					return rightMoments[mid + 1];
				}
				else {
					return rightMoments[mid];
				}
			}
			
			if(rightTimes[mid] < t) {
				start = mid;
				continue;
			}
			else if(rightTimes[mid] > t) {
				end = mid;
				continue;
			}
		}
	}
	
	public double totalTime() {
		return Math.max(leftTimes[leftTimes.length - 1], rightTimes[rightTimes.length - 1]);
	}
	public BezierPath getPath() {
		return path;
	}
	public Vec2D pathAt(double t) {
		return path.at(t);
	}
}
