package robot.pathfinder;

import robot.pathfinder.bezier.BezierPath;
import robot.pathfinder.math.MathUtils;

public class BezierTrajectory {

	//
	BezierPath path;
	Segment[] segments;
	double maxVel;
	double maxAccel;
	double t_delta;
	
	public BezierTrajectory(Waypoint[] waypoints, double alpha, double maxVel, double maxAccel, int segs) {
		path = new BezierPath(waypoints, alpha);
		this.maxVel = maxVel;
		this.maxAccel = maxAccel;
		
		t_delta = 1.0 / segs;
		segments = new Segment[segs];
		double t = 0;
		for(int i = 0; i < segs; i ++) {
			double start = i * t_delta;
			double end = start + t_delta;
			segments[i] = new Segment(start, end);
			double mid = segments[i].getMid();
			
			double r = MathUtils.getCircleFromPoints(path.at(start), path.at(end), path.at(mid)).getRadius();
			
		}
	}
}
