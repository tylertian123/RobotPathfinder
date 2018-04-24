package robot.pathfinder;

import robot.pathfinder.spline.CubicSplineSegment;

public class RobotPath {
	
	CubicSplineSegment[] xSegs;
	CubicSplineSegment[] ySegs;
	
	double scalingFactor;
	
	public RobotPath(double maxDeriv, Waypoint[] waypoints) {
		if(waypoints.length < 2) {
			throw new IllegalArgumentException("Too few waypoints");
		}
		
		xSegs = new CubicSplineSegment[waypoints.length - 1];
		ySegs = new CubicSplineSegment[waypoints.length - 1];
		
		double[] dx = new double[waypoints.length];
		double[] dy = new double[waypoints.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			double c = Math.tan(waypoints[i].getHeading());
			if(Math.abs(c) > 1) {
				dx[i] = maxDeriv / c;
			}
			else {
				dx[i] = maxDeriv;
			}
			dy[i] = Math.copySign(dx[i] * c, c);
		}
		
		for(int i = 0; i < waypoints.length - 1; i ++) {
			double[][] matX = new double[][] {
				{ Math.pow(0, 3), Math.pow(0, 2), 0, 1, waypoints[i].getX() },
				{ Math.pow(1, 3), Math.pow(1, 2), 1, 1, waypoints[i + 1].getX() },
				{ 3 * Math.pow(0, 2), 2 * 0, 1, 0, dx[i] },
				{ 3 * Math.pow(1, 2), 2 * 1, 1, 0, dx[i + 1] },
			};
			double[][] matY = new double[][] {
				{ Math.pow(0, 3), Math.pow(0, 2), 0, 1, waypoints[i].getY() },
				{ Math.pow(1, 3), Math.pow(1, 2), 1, 1, waypoints[i + 1].getY() },
				{ 3 * Math.pow(0, 2), 2 * 0, 1, 0, dy[i] },
				{ 3 * Math.pow(1, 2), 2 * 1, 1, 0, dy[i + 1] },
			};
			xSegs[i] = new CubicSplineSegment(Solver.solve(matX), 0, 1);
			ySegs[i] = new CubicSplineSegment(Solver.solve(matY), 0, 1);
		}
		scalingFactor = waypoints.length - 1;
	}
	
	public Waypoint at(double t) {
		t *= scalingFactor;
		
		double localT = t - Math.floor(t);
		
		return new Waypoint(xSegs[(int) Math.floor(t)].at(localT), ySegs[(int) Math.floor(t)].at(localT));
	}
	public Waypoint derivAt(double t) {
		t *= scalingFactor;
		
		double localT = t - Math.floor(t);
		
		return new Waypoint(xSegs[(int) Math.floor(t)].derivAt(localT), ySegs[(int) Math.floor(t)].derivAt(localT));
	}
	public Waypoint secondDerivAt(double t) {
		t *= scalingFactor;
		
		double localT = t - Math.floor(t);
		
		return new Waypoint(xSegs[(int) Math.floor(t)].secondDerivAt(localT), ySegs[(int) Math.floor(t)].secondDerivAt(localT));
	}
}
