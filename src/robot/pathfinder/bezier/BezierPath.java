package robot.pathfinder.bezier;

import robot.pathfinder.Waypoint;
import robot.pathfinder.math.Vec2D;

public class BezierPath {
	
	Bezier[] beziers;
	
	Vec2D lastPoint;
	double currentDist = 0;
	double currentT = 0;
	
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
		lastPoint = at(0);
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
}
