package robot.pathfinder.spline;

import robot.pathfinder.Solver;
import robot.pathfinder.Waypoint;

public class CubicSpline {
	
	CubicSplineSegment[] segments;
	double[] bounds;
	Waypoint[] waypoints;
	
	public CubicSpline(Waypoint... waypoints) {
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
		segments = new CubicSplineSegment[waypoints.length - 1];
		//Our equation is f(x) = ax^3 + bx^2 + cx + d
		//f'(x) = 3ax^2 + 2bx + c
		//f''(x) = 6ax + 2b
		//Put into a matrix to solve
		/*
		 * The overall path is consisted of multiple segments, each its own cubic function.
		 * The functions are fitted so that:
		 * 1. The derivative at the very first waypoint is 0
		 * 2. The derivative at the start of any subsequent segment is the same as the derivative of the end
		 * of the previous segment
		 * 3. The value of the function passes through the waypoints
		 * 4. The second derivative at the very first waypoint is 0
		 * 5. If the segment is not the last segment, its second derivative at its start must be the same
		 * as the second derivative at the end of the previous segment
		 * 6. If the segment is the last segment, its derivative at its end must be 0
		 * 
		 * For the first spline:
		 * 	| 	6x0		2		0		0		0	|
		 * 	| 	x0^3	x0^2	x0		1		y0	|
		 * 	|	x1^3	x1^2	x1		1		y1	|
		 * 	|	3x0^2	2x0		1		0		0	|
		 * 
		 * For any subsequent spline:
		 * 	|	6xn-1	2		0		0	f''(xn-1)|
		 * 	|	xn-1^3	xn-1^2	xn-1	1	yn-1	|
		 * 	|	xn^3	xn^2	xn		1	yn		|
		 * 	|	3xn-1^2	2xn-1	1		0	f'(xn-1)|
		 * 
		 * The last spline segment is similar to the previous two, but instead of having its second derivative
		 * match the last segment, it has its first derivative at the endpoint being zero.
		 * This is done because if working with robots, a non zero first derivative at the end of the path
		 * would mean the robot's speed is not 0 when we're finished.
		 */
		if(segments.length > 1) {
			double[][] firstSegmentMat = new double[][] {
				//Second derivative at x0 equals 0
				{ 6 * bounds[0], 2, 0, 0, 0 },
				//Meets waypoints
				{ Math.pow(bounds[0], 3), Math.pow(bounds[0], 2), bounds[0], 1, waypoints[0].getY() },
				{ Math.pow(bounds[1], 3), Math.pow(bounds[1], 2), bounds[1], 1, waypoints[1].getY() },
				//First derivative at x0 equals 0
				{ 3 * Math.pow(bounds[0], 2), 2 * bounds[0], 1, 0, 0},
			};
			//Solve and construct our first spline
			segments[0] = new CubicSplineSegment(Solver.solve(firstSegmentMat), bounds[0], bounds[1]);
			//Now construct subsequent segments
			for(int i = 1; i < segments.length; i ++) {
				//Check if we are on the last segment
				if(i != segments.length - 1) {
					double[][] segmentMat = new double[][] {
						//Second derivative at start equals previous second derivative
						{ 6 * bounds[i], 2, 0, 0, segments[i - 1].secondDerivAt(segments[i - 1].getEnd()) },
						//Meets waypoints
						{ Math.pow(bounds[i], 3), Math.pow(bounds[i], 2), bounds[i], 1, waypoints[i].getY() },
						{ Math.pow(bounds[i + 1], 3), Math.pow(bounds[i + 1], 2), bounds[i + 1], 1, waypoints[i + 1].getY() },
						//First derivative at start equals previous first derivative
						{ 3 * Math.pow(bounds[i], 2), 2 * bounds[i], 1, 0, segments[i - 1].derivAt(segments[i - 1].getEnd()) },
					};
					segments[i] = new CubicSplineSegment(Solver.solve(segmentMat), bounds[i], bounds[i + 1]);
				}
				else {
					double[][] segmentMat = new double[][] {
						//First derivative at end equals 0
						{ 3 * Math.pow(bounds[i + 1], 2), 2 * bounds[i + 1], 1, 0, 0 },
						//Meets waypoints
						{ Math.pow(bounds[i], 3), Math.pow(bounds[i], 2), bounds[i], 1, waypoints[i].getY() },
						{ Math.pow(bounds[i + 1], 3), Math.pow(bounds[i + 1], 2), bounds[i + 1], 1, waypoints[i + 1].getY() },
						//First derivative at start equals previous first derivative
						{ 3 * Math.pow(bounds[i], 2), 2 * bounds[i], 1, 0, segments[i - 1].derivAt(segments[i - 1].getEnd()) },
					};
					segments[i] = new CubicSplineSegment(Solver.solve(segmentMat), bounds[i], bounds[i + 1]);
				}
			}
		}
		else {
			//Since there's only one segment, this is both the beginning and the end
			double[][] firstSegmentMat = new double[][] {
				//First derivative at x1 equals 0
				{ 3 * Math.pow(bounds[1], 2), 2 * bounds[1], 1, 0, 0},
				//Meets waypoints
				{ Math.pow(bounds[0], 3), Math.pow(bounds[0], 2), bounds[0], 1, waypoints[0].getY() },
				{ Math.pow(bounds[1], 3), Math.pow(bounds[1], 2), bounds[1], 1, waypoints[1].getY() },
				//First derivative at x0 equals 0
				{ 3 * Math.pow(bounds[0], 2), 2 * bounds[0], 1, 0, 0 },
			};
			//Solve and construct our first spline
			segments[0] = new CubicSplineSegment(Solver.solve(firstSegmentMat), bounds[0], bounds[1]);
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
