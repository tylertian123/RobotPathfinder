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
			double dt = segments[i - 1].end - segments[i - 1].start;
			double lastDist = path.getIntegratedLen();
			double dist = path.integrateLen(dt);
			double distDiff = dist - lastDist;
			double maxReachableVelo = Math.sqrt(Math.pow(moments[i - 1].getVelo(), 2) + 2 * maxAccel * distDiff);
			double velo;
			
			if(maxReachableVelo > segments[i - 1].getMaxVelo()) {
				velo = segments[i - 1].getMaxVelo();
				moments[i - 1].setAccel(0);
			}
			else {
				velo = maxReachableVelo;
				moments[i - 1].setAccel(maxAccel);
			}
			moments[i] = new Moment(dist, velo, 0);
			moments[i].theoreticalMax = segments[i - 1].getMaxVelo();
		}
		//Backwards pass
		moments[segments.length].setVelo(0);
		moments[segments.length].setAccel(0);
		for(int i = segments.length - 1; i >= 0; i --) {
			double dt = segments[i].end - segments[i].start;
			double distDiff = moments[i + 1].getDist() - moments[i].getDist();
			double maxReachableVelo = Math.sqrt(Math.pow(moments[i + 1].getVelo(), 2) + 2 * maxAccel * distDiff);
			double velo;
			
			if(maxReachableVelo > moments[i].getVelo()) {
				velo = moments[i].getVelo();
			}
			else {
				velo = maxReachableVelo;
				moments[i].setAccel(-maxAccel);
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
			if(Double.isNaN(dt))
				System.out.printf("Accel: %f, Velo: %f, Dist: %f\n", accel, vel, distDiff);
			moments[i].setT(moments[i - 1].getT() + dt);
			timestamps[i] = moments[i].getT();
		}
	}
	
	public Moment getMoment(double t) {
		//Use binary search
		int start = 0;
		int end = timestamps.length - 1;
		int mid;
		
		while(true) {
			mid = (start + end) / 2;
			if(timestamps[mid] == t) 
				return moments[mid];
			if(mid == timestamps.length - 1)
				return moments[mid];
			if(timestamps[mid] <= t && timestamps[mid + 1] >= t) {
				if(Math.abs(t - timestamps[mid]) > Math.abs(t - timestamps[mid + 1])) {
					return moments[mid + 1];
				}
				else {
					return moments[mid];
				}
			}
			
			if(timestamps[mid] < t) {
				start = mid;
				continue;
			}
			else if(timestamps[mid] > t) {
				end = mid;
				continue;
			}
		}
	}
	public Vec2D pathAt(double t) {
		return path.at(t);
	}
	
	public double totalTime() {
		return timestamps[timestamps.length - 1];
	}
}
