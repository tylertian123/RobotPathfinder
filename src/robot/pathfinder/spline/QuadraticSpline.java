package robot.pathfinder.spline;

import robot.pathfinder.Solver;
import robot.pathfinder.Waypoint;

public class QuadraticSpline {
	
	QuadraticSplineSegment[] segments;
	double[] bounds;
	Waypoint[] waypoints;
	
	public QuadraticSpline(Waypoint... waypoints) {
		if(waypoints.length < 2) {
			throw new IllegalArgumentException("Too few waypoints");
		}
		this.waypoints = waypoints;
		
		//Initialize the bounds of different pieces
		bounds = new double[waypoints.length];
		for(int i = 0; i < waypoints.length; i ++) {
			bounds[i] = waypoints[i].getX();
		}
		
		//Calculate all spline segments
		//Start with the first one since it's special
		segments = new QuadraticSplineSegment[waypoints.length - 1];
		//Simplified version of the cubic spline
		double[][] firstSegmentMat = new double[][] {
			//Meets waypoints
			{ Math.pow(bounds[0], 2), bounds[0], 1, waypoints[0].getY() },
			{ Math.pow(bounds[1], 2), bounds[1], 1, waypoints[1].getY() },
			//Starting first derivative is 0
			{ 2 * bounds[0], 1, 0, 0 },
		};
		//Solve and construct our first spline
		segments[0] = new QuadraticSplineSegment(Solver.solve(firstSegmentMat), bounds[0], bounds[1]);
		//Now construct subsequent segments
		for(int i = 1; i < segments.length; i ++) {
			double[][] segmentMat = new double[][] {
				//Meets waypoints
				{ Math.pow(bounds[i], 2), bounds[i], 1, waypoints[i].getY() },
				{ Math.pow(bounds[i + 1], 2), bounds[i + 1], 1, waypoints[i + 1].getY() },
				//Derivative matches previous
				{ 2 * bounds[i], 1, 0, segments[i - 1].derivAt(segments[i - 1].getEnd())},
			};
			segments[i] = new QuadraticSplineSegment(Solver.solve(segmentMat), bounds[i], bounds[i + 1]);
		}
	}
	
	public double getStart() {
		return bounds[0];
	}
	public double getEnd() {
		return bounds[bounds.length - 1];
	}
	
	public boolean includes(double t) {
		return t >= bounds[0] && t <= bounds[bounds.length - 1];
	}
	
	public double at(double t) {
		if(!includes(t))
			throw new IllegalArgumentException("Out of range");
		for(int i = 0; i < bounds.length - 1; i ++) {
			if(t >= bounds[i] && t <= bounds[i + 1]) {
				return segments[i].at(t);
			}
		}
		return segments[segments.length - 1].at(t);
	}
	public double derivAt(double t) {
		if(!includes(t))
			throw new IllegalArgumentException("Out of range");
		for(int i = 0; i < bounds.length - 1; i ++) {
			if(t >= bounds[i] && t <= bounds[i + 1]) {
				return segments[i].derivAt(t);
			}
		}
		return segments[segments.length - 1].derivAt(t);
	}
	public double secondDerivAt(double t) {
		if(!includes(t))
			throw new IllegalArgumentException("Out of range");
		for(int i = 0; i < bounds.length - 1; i ++) {
			if(t >= bounds[i] && t <= bounds[i + 1]) {
				return segments[i].secondDerivAt(t);
			}
		}
		return segments[segments.length - 1].secondDerivAt(t);
	}
}
