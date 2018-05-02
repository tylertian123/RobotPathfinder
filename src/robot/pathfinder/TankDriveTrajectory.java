package robot.pathfinder;

import robot.pathfinder.bezier.BezierPath;
import robot.pathfinder.math.Circle;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.Moment;

public class TankDriveTrajectory {
	
	BezierPath path;
	TankDrivePathSegment[] segments;
	Moment[] leftMoments, rightMoments;
	double leftTimes, rightTimes;
	
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
		
		Vec2D lastEnd = path.at(0);
		for(int i = 0; i < segmentCount; i ++) {
			double start = i * t_delta;
			double mid = start + t_delta2;
			double end = start + t_delta;
			segments[i] = new TankDrivePathSegment(start, end);
			
			Vec2D endPointVector = path.at(end);
			Vec2D midPointVector = path.at(mid);
			Circle c = MathUtils.getCircleFromPoints(lastEnd, midPointVector, endPointVector);
			double r = c.getRadius();
			Vec2D center = c.getCenter();
			
			Vec2D deriv = path.derivAt(mid);
			double currentDirection = Math.atan2(deriv.getY(), deriv.getX());
			double centerDirection = Math.atan2(center.getY() - midPointVector.getY(), center.getX() - midPointVector.getX());
			int sign = centerDirection > currentDirection ? 1 : -1;
			
			double vMax = (2 * maxVel) / (2 + baseWidth / r);
			double omega = vMax / r * sign;
			
			double vMaxRight = (2 * vMax + omega * baseWidth) / 2;
			double vMaxLeft = 2 * vMax - vMaxRight;
			
			segments[i].setMaxVelocities(vMaxLeft, vMaxRight);
		}
		
		leftMoments = new Moment[segmentCount + 1];
		rightMoments = new Moment[segmentCount + 1];
		
		leftMoments[0] = new Moment(0, 0, 0);
		rightMoments[0] = new Moment(0, 0, 0);
		
		for(int i = 1; i < segmentCount + 1; i ++) {
			double dt = segments[i - 1].getEnd() - segments[i - 1].getEnd();
			double lastDist = path.getIntegratedLen();//TODO Fix
			double currentDist = path.integrateLen(dt);
			double distDiff = currentDist - lastDist;
			
			//Use the fourth kinematic equation to figure out the maximum reachable velocity for each side
			double leftMax = Math.sqrt(Math.pow(leftMoments[i - 1].getVelocity(), 2) + 2 * maxAccel * distDiff);
			double rightMax = Math.sqrt(Math.pow(rightMoments[i - 1].getVelocity(), 2) + 2 * maxAccel * distDiff);
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
			
			leftMoments[i] = new Moment(currentDist, leftVel, 0);
			rightMoments[i] = new Moment(currentDist, rightVel, 0);
		}
	}
}
