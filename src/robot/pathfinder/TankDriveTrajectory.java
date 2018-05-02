package robot.pathfinder;

import robot.pathfinder.bezier.BezierPath;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.Moment;

/**
 * A trajectory for a tank drive robot.<br>
 * <br>
 * This class written mainly based on the Cheesy Poofs' video, with some changes:<br>
 * https://youtu.be/8319J1BEHwM
 * @author Tyler
 *
 */
public class TankDriveTrajectory {
	
	//The internal path that this trajectory is based on
	BezierPath path;
	//The path is divided into segments which are stored here
	TankDrivePathSegment[] segments;
	//"Moments" are generated for left and right separately
	Moment[] leftMoments, rightMoments;
	//Used to find the correct moment for a given time
	double[] leftTimes, rightTimes;
	
	final double maxVel, maxAccel, baseWidth;
	
	/**
	 * Generates a trajectory based on a number of parameters.<br>
	 * <br>
	 * The units for the parameters must be consistent with each other. For example, if maximum velocity is in feet/second, 
	 * then maximum acceleration must be in feet/second^2, base width must be in feet, and the units for waypoints also in feet.<br>
	 * <br>
	 * Note this process can take up to half a second, depending on the number of segments.
	 * 
	 * @param waypoints - The waypoints the path has to travel through
	 * @param maxVelocity - The maximum velocity of the robot
	 * @param maxAcceleration - The maximum acceleration of the robot
	 * @param baseWidth - The width of the base plate of the robot (distance between left side wheels and right side wheels)
	 * @param alpha - Path smoothness constant. A higher alpha makes for smoother turns, but longer distance for the robot to travel
	 * @param segmentCount - How many segments the path is split into. A higher count makes the path more precise, but requires more time to generate
	 */
	public TankDriveTrajectory(Waypoint[] waypoints, double maxVelocity, double maxAcceleration, double baseWidth, double alpha, int segmentCount) {
		maxVel = maxVelocity;
		maxAccel = maxAcceleration;
		this.baseWidth = baseWidth;
		
		//Generate the path
		path = new BezierPath(waypoints, alpha);
		path.setBaseRadius(baseWidth / 2);
		double t_delta = 1.0 / segmentCount;
		double t_delta2 = t_delta / 2;
		
		segments = new TankDrivePathSegment[segmentCount];
		//Split the path into segments
		for(int i = 0; i < segmentCount; i ++) {
			//For each segment, compute the starting time, ending time and middle time
			double start = i * t_delta;
			double mid = start + t_delta2;
			double end = start + t_delta;
			segments[i] = new TankDrivePathSegment(start, end);
			
			//Use the curvature formula to compute curvature for the midpoint, and take its inverse to find the radius
			Vec2D deriv = path.derivAt(mid);
			double xDeriv = deriv.getX();
			double yDeriv = deriv.getY();
			Vec2D secondDeriv = path.secondDerivAt(mid);
			double xSecondDeriv = secondDeriv.getX();
			double ySecondDeriv = secondDeriv.getY();
			double curvature = (xDeriv * ySecondDeriv - yDeriv * xSecondDeriv) /
					Math.pow(xDeriv * xDeriv + yDeriv * yDeriv, 3.0/2.0);
			
			//R is always positive
			double r = Math.abs(1 / curvature);
			//Compute max speed of entire robot with a positive R
			double vMax = (2 * maxVel) / (2 + baseWidth / r);
			//Compute omega with a signed R
			//Since R is not signed, the sign has to be copied from the curvature
			double omega = Math.copySign(vMax / r, curvature);
			
			//Compute left and right maximum velocities
			double vMaxRight = (2 * vMax + omega * baseWidth) / 2;
			double vMaxLeft = 2 * vMax - vMaxRight;
			
			segments[i].setMaxVelocities(vMaxLeft, vMaxRight);
		}
		
		//Initialize "moments"
		leftMoments = new Moment[segmentCount + 1];
		rightMoments = new Moment[segmentCount + 1];
		
		leftMoments[0] = new Moment(0, 0, 0);
		rightMoments[0] = new Moment(0, 0, 0);
		
		//Forwards pass
		for(int i = 1; i < segmentCount + 1; i ++) {
			//Get the difference in t so we can integrate the paths
			double dt = segments[i - 1].getEnd() - segments[i - 1].getStart();
			
			//Integrate the path of both sides separately, since they will have different lengths
			double[] lastDists = path.getIntegratedWheelLens();
			double[] currentDists = path.integrateWheelLens(dt);
			double distDiffLeft = currentDists[0] - lastDists[0];
			double distDiffRight = currentDists[1] - lastDists[1];
			
			//Use the fourth kinematic equation to figure out the maximum reachable velocity for each side,
			//while obeying constraints for max acceleration
			double leftMax = Math.sqrt(Math.pow(leftMoments[i - 1].getVelocity(), 2) + 2 * maxAccel * distDiffLeft);
			double rightMax = Math.sqrt(Math.pow(rightMoments[i - 1].getVelocity(), 2) + 2 * maxAccel * distDiffRight);
			double leftVel, rightVel;
			
			//Check if our maximum reachable velocity is above our maximum velocity
			if(leftMax > segments[i - 1].getLeftMaxVelocity()) {
				leftVel = segments[i - 1].getLeftMaxVelocity();
				//If yes, then do not accelerate, so set the acceleration for the last moment to be 0
				leftMoments[i - 1].setAcceleration(0);
			}
			else {
				leftVel = leftMax;
				//Otherwise set the acceleration of the last moment to the maximum acceleration
				leftMoments[i - 1].setAcceleration(maxAccel);
			}
			//Do the same thing for the right
			if(rightMax > segments[i - 1].getRightMaxVelocity()) {
				rightVel = segments[i - 1].getRightMaxVelocity();
				rightMoments[i - 1].setAcceleration(0);
			}
			else {
				rightVel = rightMax;
				rightMoments[i - 1].setAcceleration(maxAccel);
			}
			
			//Create our Moment objects with the desired distance and velocity and a default acceleration of 0
			//which will be changed later if necessary.
			leftMoments[i] = new Moment(currentDists[0], leftVel, 0);
			rightMoments[i] = new Moment(currentDists[1], rightVel, 0);
		}
		
		//Prepare for backwards pass
		leftMoments[segments.length].setVelocity(0);
		leftMoments[segments.length].setAcceleration(0);
		rightMoments[segments.length].setVelocity(0);
		rightMoments[segments.length].setAcceleration(0);
		//Backwards pass
		for(int i = segments.length - 1; i >= 0; i --) {
			//Since we already have our desired distances generated, just retrieve them from the Moment objects
			double leftDistDiff = leftMoments[i + 1].getDistance() - leftMoments[i].getDistance();
			double rightDistDiff = rightMoments[i + 1].getDistance() - rightMoments[i].getDistance();
			
			//Use the fourth kinematic equation to find out the maximum reachable speed
			double leftMax = Math.sqrt(Math.pow(leftMoments[i + 1].getVelocity(), 2) + 2 * maxAccel * leftDistDiff);
			double rightMax = Math.sqrt(Math.pow(rightMoments[i + 1].getVelocity(), 2) + 2 * maxAccel * rightDistDiff);
			double leftVel, rightVel;
			
			//Compare our maximum reachable velocity to the velocity generated by the forwards pass
			if(leftMax > leftMoments[i].getVelocity()) {
				//If it's higher, don't do anything, as the forwards pass should have already settled things
				leftVel = leftMoments[i].getVelocity();
			}
			else {
				//If it's lower, then correct it, and set this moment to decelerate
				leftVel = leftMax;
				leftMoments[i].setAcceleration(-maxAccel);
			}
			leftMoments[i].setVelocity(leftVel);
			//Do the same for the right
			if(rightMax > rightMoments[i].getVelocity()) {
				rightVel = rightMoments[i].getVelocity();
			}
			else {
				rightVel = rightMax;
				rightMoments[i].setAcceleration(-maxAccel);
			}
			rightMoments[i].setVelocity(rightVel);
		}
		
		//Initialize
		leftTimes = new double[leftMoments.length];
		rightTimes = new double[rightMoments.length];
		leftTimes[0] = 0;
		rightTimes[0] = 0;
		//Now that we have the desired distances, velocities and accelerations, we need to assign each moment a time
		for(int i = 1; i < leftMoments.length; i ++) {
			//Get the desired distance, velocity and acceleration separately
			double distDiffLeft = leftMoments[i].getDistance() - leftMoments[i - 1].getDistance();
			double distDiffRight = rightMoments[i].getDistance() - rightMoments[i - 1].getDistance();
			double leftVel = leftMoments[i - 1].getVelocity();
			double rightVel = rightMoments[i - 1].getVelocity();
			double leftAccel = leftMoments[i - 1].getAcceleration();
			double rightAccel = rightMoments[i - 1].getAcceleration();
			
			//Use the second kinematic formula to find the amount of time needed to reach the desired distance
			//This time is then used to give every Moment a timestamp (separately for left and right)
			double dtLeft = MathUtils.findPositiveQuadraticRoot(leftAccel / 2, leftVel, -distDiffLeft);
			double dtRight = MathUtils.findPositiveQuadraticRoot(rightAccel / 2, rightVel, -distDiffRight);
			//
			if(Double.isNaN(dtLeft) || Double.isInfinite(dtLeft)) {
				System.out.printf("*LEFT* Accel: %f, Velo: %f, Dist: %f, Iteration: %d\n", leftAccel, leftVel, distDiffLeft, i);
				throw new PathGenerationException("Path is impossible");
			}
			if(Double.isNaN(dtRight) || Double.isInfinite(dtRight)) {
				System.out.printf("*RIGHT* Accel: %f, Velo: %f, Dist: %f, Iteration: %d\n", rightAccel, rightVel, distDiffRight, i);
				throw new PathGenerationException("Path is impossible");
			}
			
			leftMoments[i].setTime(leftMoments[i - 1].getTime() + dtLeft);
			rightMoments[i].setTime(rightMoments[i - 1].getTime() + dtRight);
			leftTimes[i] = leftMoments[i].getTime();
			rightTimes[i] = rightMoments[i].getTime();
			//System.out.printf("l: %f, r: %f\n", leftTimes[i], rightTimes[i]);
		}
	}
	
	public Moment getLeft(double t) {
		int start = 0;
		int end = leftTimes.length - 1;
		int mid;
		
		if(t >= leftTimes[leftTimes.length - 1])
			return leftMoments[leftMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			if(leftTimes[mid] == t)
				return leftMoments[mid];
			
			if(mid == leftTimes.length - 1)
				return leftMoments[mid];
			
			if(leftTimes[mid] <= t && leftTimes[mid + 1] >= t) {
				if(Math.abs(t - leftTimes[mid]) > Math.abs(t - leftTimes[mid + 1])) {
					return leftMoments[mid + 1];
				}
				else {
					return leftMoments[mid];
				}
			}
			
			if(leftTimes[mid] < t) {
				start = mid;
				continue;
			}
			else if(leftTimes[mid] > t) {
				end = mid;
				continue;
			}
		}
	}
	public Moment getRight(double t) {
		int start = 0;
		int end = rightTimes.length - 1;
		int mid;
		
		if(t >= rightTimes[rightTimes.length - 1])
			return rightMoments[rightMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			if(rightTimes[mid] == t)
				return rightMoments[mid];
			
			if(mid == rightTimes.length - 1)
				return rightMoments[mid];
			
			if(rightTimes[mid] <= t && rightTimes[mid + 1] >= t) {
				if(Math.abs(t - rightTimes[mid]) > Math.abs(t - rightTimes[mid + 1])) {
					return rightMoments[mid + 1];
				}
				else {
					return rightMoments[mid];
				}
			}
			
			if(rightTimes[mid] < t) {
				start = mid;
				continue;
			}
			else if(rightTimes[mid] > t) {
				end = mid;
				continue;
			}
		}
	}
	
	public double totalTime() {
		return Math.max(leftTimes[leftTimes.length - 1], rightTimes[rightTimes.length - 1]);
	}
	public BezierPath getPath() {
		return path;
	}
	public Vec2D pathAt(double t) {
		return path.at(t);
	}
}
