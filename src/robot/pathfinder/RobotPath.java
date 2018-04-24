package robot.pathfinder;

import robot.pathfinder.spline.CubicSplineSegment;

public class RobotPath {
	
	CubicSplineSegment[] xSegs;
	CubicSplineSegment[] ySegs;
	
	double scalingFactor;
	
	double baseRadius;
	
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
	public RobotPath(double maxDeriv, Waypoint[] waypoints, double baseRadius) {
		this(maxDeriv, waypoints);
		this.baseRadius = baseRadius;
	}
	
	public void setBaseRadius(double baseRadius) {
		this.baseRadius = baseRadius;
	}
	
	public Vec2D at(double t) {
		t *= scalingFactor;
		
		double localT = t - Math.floor(t);
		
		return new Vec2D(xSegs[(int) Math.floor(t)].at(localT), ySegs[(int) Math.floor(t)].at(localT));
	}
	public Vec2D derivAt(double t) {
		t *= scalingFactor;
		
		double localT = t - Math.floor(t);
		
		return new Vec2D(xSegs[(int) Math.floor(t)].derivAt(localT), ySegs[(int) Math.floor(t)].derivAt(localT));
	}
	public Vec2D secondDerivAt(double t) {
		t *= scalingFactor;
		
		double localT = t - Math.floor(t);
		
		return new Vec2D(xSegs[(int) Math.floor(t)].secondDerivAt(localT), ySegs[(int) Math.floor(t)].secondDerivAt(localT));
	}
	
	public Vec2D leftWheelAt(double t) {
		Vec2D pos = at(t);
		Vec2D vel = derivAt(t);
		
		double heading = Math.atan2(vel.getY(), vel.getX());
		heading += Math.PI / 2;
		
		return new Vec2D(pos.getX() + Math.cos(heading) * baseRadius, pos.getY() + Math.sin(heading) * baseRadius);
	}
	public Vec2D rightWheelAt(double t) {
		Vec2D pos = at(t);
		Vec2D vel = derivAt(t);
		
		if(!Double.isFinite(vel.getY() / vel.getX()))
			System.out.println("Infinite");
		
		double heading = Math.atan2(vel.getY(), vel.getX());
		heading -= Math.PI / 2;
		
		return new Vec2D(pos.getX() + Math.cos(heading) * baseRadius, pos.getY() + Math.sin(heading) * baseRadius);
	}
}
