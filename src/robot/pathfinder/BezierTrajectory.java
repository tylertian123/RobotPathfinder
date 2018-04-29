package robot.pathfinder;

import robot.pathfinder.bezier.BezierPath;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;

public class BezierTrajectory {

	//
	BezierPath path;
	Segment[] segments;
	Moment[] moments;
	double[] timestamps;
	double maxVel;
	double maxAccel;
	double t_delta;
	double baseWidth;
	
	public BezierTrajectory(Waypoint[] waypoints, double alpha, double maxVel, double maxAccel, double baseWidth, int segs) {
		path = new BezierPath(waypoints, alpha);
		this.maxVel = maxVel;
		this.maxAccel = maxAccel;
		this.baseWidth = baseWidth;
		
		t_delta = 1.0 / segs;
		segments = new Segment[segs];
		//Store last result to speed up computing
		Vec2D lastEndVec = path.at(0);
		for(int i = 0; i < segs; i ++) {
			double start = i * t_delta;
			double end = start + t_delta;
			double mid = start + t_delta / 2;
			segments[i] = new Segment(start, end);
			
			Vec2D endVec = path.at(end);
			double r = MathUtils.getCircleFromPoints(lastEndVec, path.at(mid), endVec).getRadius();
			lastEndVec = endVec;
			segments[i].setMaxVelo(Math.min(maxVel, r / baseWidth));
			segments[i].setR(r);
		}
		moments = new Moment[segments.length + 1];
		
		//Forwards pass
		moments[0] = new Moment(0, 0, 0, 0);
		for(int i = 1; i < segments.length + 1; i ++) {
			double dt = segments[i].end - segments[i].start;
			double dist = path.integrateLen(dt);
			double maxReachableVelo = moments[i - 1].getVelo() + maxAccel * dt;
			double velo;
			
			if(maxReachableVelo > segments[i].getMaxVelo()) {
				velo = segments[i].getMaxVelo();
			}
			else {
				velo = maxReachableVelo;
				moments[i - 1].setAccel(maxAccel);
			}
			moments[i] = new Moment(dist, velo, 0);
		}
		//Backwards pass
		moments[segments.length].setVelo(0);
		moments[segments.length].setAccel(0);
		for(int i = segments.length - 1; i >= 0; i --) {
			double dt = segments[i].end - segments[i].start;
			double maxReachableVelo = moments[i + 1].getVelo() + maxAccel * dt;
			double velo;
			
			if(maxReachableVelo > segments[i].getMaxVelo()) {
				velo = segments[i].getMaxVelo();
			}
			else {
				velo = maxReachableVelo;
				moments[i - 1].setAccel(-maxAccel);
			}
			moments[i].setVelo(velo);
		}
		//Add a timestamp
		timestamps = new double[moments.length];
		timestamps[0] = 0;
		for(int i = 1; i < moments.length; i ++) {
			double distDiff = moments[i].getDist() - moments[i - 1].getDist();
			double vel = moments[i - 1].getVelo();
			double accel = moments[i - 1].getAccel();
			
			double dt = MathUtils.findPositiveQuadraticRoot(accel / 2, vel, -distDiff);
			moments[i].setT(moments[i - 1].getT() + dt);
			timestamps[i] = moments[i].getT();
		}
	}
	
	public Moment getMoment(double t) {
		for(int i = 0; i < timestamps.length; i ++) {
			if(i == timestamps.length - 1)
				return moments[i];
			if(t <= timestamps[i + 1]) {
				if(t - timestamps[i] > timestamps[i + 1] - t) {
					return moments[i];
				}
				else {
					return moments[i + 1];
				}
			}
		}
		//Just so that the compiler stops complaining
		return null;
	}
}
