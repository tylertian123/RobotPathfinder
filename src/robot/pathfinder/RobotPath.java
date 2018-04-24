package robot.pathfinder;

import robot.pathfinder.spline.CubicSplineSegment;

public class RobotPath {
	
	CubicSplineSegment[] xSegs;
	CubicSplineSegment[] ySegs;
	double[] bounds;
	
	double scalingFactor;
	
	public RobotPath(double maxDeriv, Waypoint[] waypoints, double[] headings) {
		if(waypoints.length < 2) {
			throw new IllegalArgumentException("Too few waypoints");
		}
		if(waypoints.length != headings.length) {
			throw new IllegalArgumentException("The number of waypoints does not match the number of headings");
		}
		
		bounds = new double[waypoints.length];
		for(int i = 0; i < waypoints.length; i ++) {
			bounds[i] = i;
		}
		
		xSegs = new CubicSplineSegment[waypoints.length - 1];
		ySegs = new CubicSplineSegment[waypoints.length - 1];
		
		double[] dx = new double[waypoints.length];
		double[] dy = new double[waypoints.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			double c = Math.tan(headings[i]);
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
				{ Math.pow(bounds[i], 3), Math.pow(bounds[i], 2), bounds[i], 1, waypoints[i].getX() },
				{ Math.pow(bounds[i + 1], 3), Math.pow(bounds[i + 1], 2), bounds[i + 1], 1, waypoints[i + 1].getX() },
				{ 3 * Math.pow(bounds[i], 2), 2 * bounds[i], 1, 0, dx[i] },
				{ 3 * Math.pow(bounds[i + 1], 2), 2 * bounds[i + 1], 1, 0, dx[i + 1] },
			};
			double[][] matY = new double[][] {
				{ Math.pow(bounds[i], 3), Math.pow(bounds[i], 2), bounds[i], 1, waypoints[i].getY() },
				{ Math.pow(bounds[i + 1], 3), Math.pow(bounds[i + 1], 2), bounds[i + 1], 1, waypoints[i + 1].getY() },
				{ 3 * Math.pow(bounds[i], 2), 2 * bounds[i], 1, 0, dy[i] },
				{ 3 * Math.pow(bounds[i + 1], 2), 2 * bounds[i + 1], 1, 0, dy[i + 1] },
			};
			xSegs[i] = new CubicSplineSegment(Solver.solve(matX), bounds[i], bounds[i + 1]);
			ySegs[i] = new CubicSplineSegment(Solver.solve(matY), bounds[i], bounds[i + 1]);
		}
		scalingFactor = waypoints.length - 1;
	}
	
	public Waypoint at(double t) {
		t *= scalingFactor;
		
		for(int i = 0; i < bounds.length - 1; i ++) {
			if(t >= bounds[i] && t <= bounds[i + 1]) {
				return new Waypoint(xSegs[i].at(t), ySegs[i].at(t));
			}
		}
		return new Waypoint(xSegs[xSegs.length - 1].at(t), ySegs[ySegs.length - 1].at(t));
	}
	public Waypoint derivAt(double t) {
		t *= scalingFactor;
		
		for(int i = 0; i < bounds.length - 1; i ++) {
			if(t >= bounds[i] && t <= bounds[i + 1]) {
				return new Waypoint(xSegs[i].derivAt(t), ySegs[i].derivAt(t));
			}
		}
		return new Waypoint(xSegs[xSegs.length - 1].derivAt(t), ySegs[ySegs.length - 1].derivAt(t));
	}
	public Waypoint secondDerivAt(double t) {
		t *= scalingFactor;
		
		for(int i = 0; i < bounds.length - 1; i ++) {
			if(t >= bounds[i] && t <= bounds[i + 1]) {
				return new Waypoint(xSegs[i].secondDerivAt(t), ySegs[i].secondDerivAt(t));
			}
		}
		return new Waypoint(xSegs[xSegs.length - 1].secondDerivAt(t), ySegs[ySegs.length - 1].secondDerivAt(t));
	}
}
