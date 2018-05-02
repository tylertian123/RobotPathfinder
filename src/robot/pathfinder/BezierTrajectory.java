package robot.pathfinder;

import robot.pathfinder.bezier.BezierPath;
import robot.pathfinder.math.Circle;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.old.Moment;

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
		/*ArrayList<Double> xD = new ArrayList<Double>();
		ArrayList<Double> yD = new ArrayList<Double>();
		ArrayList<Double> xV = new ArrayList<Double>();
		ArrayList<Double> yV = new ArrayList<Double>();
		ArrayList<Double> x2D = new ArrayList<Double>();
		ArrayList<Double> y2D = new ArrayList<Double>();
		ArrayList<Double> time = new ArrayList<Double>();*/
		
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
			Circle c = MathUtils.getCircleFromPoints(lastEndVec, path.at(mid), endVec);
			double r = c.getRadius();
			double curvature = 1 / r;
			
			Vec2D deriv = path.derivAt(mid);
			double heading = Math.atan2(deriv.getY(), deriv.getX());
			double centerHeading = Math.atan2(c.getCenter().getY() - lastEndVec.getY(), c.getCenter().getX() - lastEndVec.getX());
			int m = heading > centerHeading ? -1 : 1;
			/*double xDeriv = deriv.getX();
			double yDeriv = deriv.getY();
			Vec2D secondDeriv = path.secondDerivAt(mid);
			double xSecondDeriv = secondDeriv.getX();
			double ySecondDeriv = secondDeriv.getY();
			double curvature2 = (xDeriv * ySecondDeriv - yDeriv * xSecondDeriv) /
					Math.pow(xDeriv * xDeriv + yDeriv * yDeriv, 3.0/2.0);
			double r2 = 1 / curvature2;*/
			/*Vec2D val = path.at(mid);
			xV.add(val.getX());
			yV.add(val.getY());
			xD.add(xDeriv);
			yD.add(yDeriv);
			x2D.add(xSecondDeriv);
			y2D.add(ySecondDeriv);
			time.add(mid);*/
			//System.out.printf("Estimate: %f, Deriv: %f, Point: %f\n", r * m, r2, mid);
			
			lastEndVec = endVec;
			//segments[i].setMaxVelo(Math.min(maxVel, r / baseWidth));
			//segments[i].setMaxVelo((2 * maxVel - Math.abs(curvature) * baseWidth) / 2);
			segments[i].setMaxVelo((2 * maxVel) / (2 + baseWidth / r)); 
			segments[i].setR(r);
		}
		/*Plot2DPanel plot = new Plot2DPanel();
		plot.addLinePlot("X", Testing.primitiveArr(time), Testing.primitiveArr(xV));
		plot.addLinePlot("Y", Testing.primitiveArr(time), Testing.primitiveArr(yV));
		plot.addLinePlot("X Derivative", Testing.primitiveArr(time), Testing.primitiveArr(xD));
		plot.addLinePlot("Y Derivative", Testing.primitiveArr(time), Testing.primitiveArr(yD));
		plot.addLinePlot("X Second Derivative", Testing.primitiveArr(time), Testing.primitiveArr(x2D));
		plot.addLinePlot("Y Second Derivative", Testing.primitiveArr(time), Testing.primitiveArr(y2D));
		plot.setLegendOrientation("EAST");
		JFrame frame = new JFrame("Derivatives");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame.setContentPane(plot);
		frame.setVisible(true);*/
		moments = new Moment[segments.length + 1];
		
		//Forwards pass
		moments[0] = new Moment(0, 0, 0, 0);
		moments[0].setMaxVelo(segments[0].getMaxVelo());
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
