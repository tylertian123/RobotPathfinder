package robot.pathfinder.bezier;

import robot.pathfinder.Waypoint;
import robot.pathfinder.math.Vec2D;

public class BezierPath {
	
	Bezier[] beziers;
	
	Vec2D lastPoint;
	double currentDist = 0;
	double currentT = 0;
	Vec2D[] lastWheelPositions;
	double currentWheelDists[] = new double[2];
	double currentTWheels = 0;
	
	double baseRadius;
	
	public BezierPath(Waypoint[] waypoints, double alpha) {
		if(waypoints.length < 2) {
			throw new IllegalArgumentException("Not enough waypoints");
		}
		beziers = new Bezier[waypoints.length - 1];
		for(int i = 0; i < beziers.length; i ++) {
			beziers[i] = Bezier.getFromHermite(waypoints[i].getPosVec(), waypoints[i + 1].getPosVec(),
					new Vec2D(Math.cos(waypoints[i].getHeading()) * alpha, Math.sin(waypoints[i].getHeading()) * alpha),
					new Vec2D(Math.cos(waypoints[i + 1].getHeading()) * alpha, Math.sin(waypoints[i + 1].getHeading()) * alpha));
		}
		
		resetIntegration();
		resetWheelIntegration();
	}
	
	public void setBaseRadius(double b) {
		baseRadius = b;
	}
	public double getBaseRaidus() {
		return baseRadius;
	}
	
	public Vec2D at(double t) {
		t *= beziers.length;
		try {
			return beziers[(int) Math.floor(t)].at(t % 1.0);
		}
		catch(ArrayIndexOutOfBoundsException e) {
			return beziers[beziers.length - 1].at(t % 1.0);
		}
	}
	public Vec2D[] wheelsAt(double t) {
		Vec2D position = at(t);
		Vec2D derivative = derivAt(t);
		
		double heading = Math.atan2(derivative.getY(), derivative.getX());
		double leftHeading = heading + Math.PI / 2;
		double rightHeading = heading - Math.PI / 2;
		
		return new Vec2D[] {
				new Vec2D(position.getX() + Math.cos(leftHeading) * baseRadius, position.getY() + Math.sin(leftHeading) * baseRadius),
				new Vec2D(position.getX() + Math.cos(rightHeading) * baseRadius, position.getY() + Math.sin(rightHeading) * baseRadius)
		};
	}
	public Vec2D derivAt(double t) {
		t *= beziers.length;
		return beziers[(int) Math.floor(t)].derivAt(t % 1.0);
	}
	public Vec2D secondDerivAt(double t) {
		t *= beziers.length;
		return beziers[(int) Math.floor(t)].secondDerivAt(t % 1.0);
	}
	
	public double integrateLen(double dt) {
		currentT += dt;
		if(currentT > 1) {
			return currentDist;
		}
		Vec2D currentPoint = at(currentT);
		currentDist += lastPoint.distTo(currentPoint);
		lastPoint = currentPoint;
		return currentDist;
	}
	public void resetIntegration() {
		currentT = 0;
		lastPoint = at(0);
		currentDist = 0;
	}
	public double getIntegratedLen() {
		return currentDist;
	}
	
	public double[] integrateWheelLens(double dt) {
		//Mystery bug
		if(currentTWheels == 0)
			resetWheelIntegration();
		currentTWheels += dt;
		if(currentTWheels > 1)
			return currentWheelDists;
		Vec2D[] currentWheels = wheelsAt(currentTWheels);
		currentWheelDists[0] += lastWheelPositions[0].distTo(currentWheels[0]);
		currentWheelDists[1] += lastWheelPositions[1].distTo(currentWheels[1]);
		lastWheelPositions = currentWheels;
		return currentWheelDists;
	}
	public void resetWheelIntegration() {
		currentTWheels = 0;
		lastWheelPositions = wheelsAt(0);
		currentWheelDists = new double[2];
	}
	public double[] getIntegratedWheelLens() {
		return currentWheelDists.clone();
	}
}
