package robot.pathfinder.tankdrive;

import robot.pathfinder.core.Moment;
import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryGenerationException;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;

/**
 * A trajectory generator for a tank drive robot.<br>
 * <br>
 * The algorithm used in this class is largely based on a video by the Cheesy Poofs (Team 254),
 * with some small modifications here and there.<br>
 * You can find the video on YouTube <a href="https://youtu.be/8319J1BEHwM">here</a>.<br>
 * <br>
 * In addition to the Poofs' algorithm, this class divides the trajectory into "moments".
 * Each "moment" is a point in time, and has a desired displacement, velocity and acceleration.
 * Using these "moments", a system can be set up that follows the desired values, thus achieving 
 * motion control.<br>
 * This class also uses Beziers to generate its path, instead of 2D Hermite spline fitting like in
 * the video.
 * 
 * @author Tyler Tian
 *
 */
public class TankDriveTrajectory {
	
	//The internal path that this trajectory is based on
	BezierPath path;
	//"Moments" are generated for left and right separately
	Moment[] leftMoments, rightMoments;
	
	//Used in solving quadratic equations
	//If |b^2-4ac| <= this number, it will be set to 0 to avoid having no real solutions
	//due to rounding errors.
	static double minUnit = 1.0e-5;
	
	/**
	 * Generates a trajectory based on a number of parameters.<br>
	 * <br>
	 * The units for the parameters must be consistent with each other. For example, if maximum velocity is in feet/second, 
	 * then maximum acceleration must be in feet/second^2, base width must be in feet, and the units for waypoints also in feet.<br>
	 * <br>
	 * Note this process can take up to half a second, depending on the number of segments.
	 * 
	 * @param waypoints The waypoints the path has to travel through
	 * @param robotSpecs A {@link robot.pathfinder.core.RobotSpecs RobotSpecs} object containing the specifications of the robot
	 * @param alpha Path smoothness constant. A higher alpha makes for smoother turns, but longer distance for the robot to travel
	 * @param segmentCount How many segments the path is split into. A higher count makes the path more precise, but requires more time to generate
	 */
	public TankDriveTrajectory(Waypoint[] waypoints, RobotSpecs robotSpecs, double alpha, int segmentCount) {
		this(waypoints, robotSpecs, alpha, segmentCount, false);
	}
	/**
	 * Generates a trajectory based on a number of parameters.<br>
	 * <br>
	 * The units for the parameters must be consistent with each other. For example, if maximum velocity is in feet/second, 
	 * then maximum acceleration must be in feet/second^2, base width must be in feet, and the units for waypoints also in feet.<br>
	 * <br>
	 * Note this process can take up to half a second, depending on the number of segments.
	 * 
	 * @param waypoints The waypoints the path has to travel through
	 * @param robotSpecs A {@link robot.pathfinder.core.RobotSpecs RobotSpecs} object containing the specifications of the robot	 
	 * @param alpha Path smoothness constant. A higher alpha makes for smoother turns, but longer distance for the robot to travel
	 * @param segmentCount How many segments the path is split into. A higher count makes the path more precise, but requires more time to generate
	 * @param surpressExceptions If set to true, an exception will <b>not</b> be thrown if the path is impossible
	 */
	public TankDriveTrajectory(Waypoint[] waypoints, RobotSpecs robotSpecs, double alpha, int segmentCount, boolean surpressExceptions) {
		this(waypoints, robotSpecs.getMaxVelocity(), robotSpecs.getMaxAcceleration(), robotSpecs.getMaxDeceleration(), robotSpecs.getMaxJerk(), robotSpecs.getBaseWidth(), alpha, segmentCount, surpressExceptions);
	}
	/**
	 * Generates a trajectory based on a number of parameters.<br>
	 * <br>
	 * The units for the parameters must be consistent with each other. For example, if maximum velocity is in feet/second, 
	 * then maximum acceleration must be in feet/second^2, base width must be in feet, and the units for waypoints also in feet.<br>
	 * <br>
	 * Note this process can take up to half a second, depending on the number of segments.
	 * 
	 * @param waypoints The waypoints the path has to travel through
	 * @param maxVelocity The maximum velocity of the robot
	 * @param maxAcceleration The maximum acceleration of the robot
	 * @param maxJerk The maximum jerk (change in acceleration) of the robot
	 * @param baseWidth The width of the base plate of the robot (distance between left side wheels and right side wheels)
	 * @param alpha Path smoothness constant. A higher alpha makes for smoother turns, but longer distance for the robot to travel
	 * @param segmentCount How many segments the path is split into. A higher count makes the path more precise, but requires more time to generate
	 */
	public TankDriveTrajectory(Waypoint[] waypoints, double maxVelocity, double maxAcceleration, double maxJerk, double baseWidth, double alpha, int segmentCount) {
		this(waypoints, maxVelocity, maxAcceleration, maxJerk, baseWidth, alpha, segmentCount, false);
	}
	/**
	 Generates a trajectory based on a number of parameters.<br>
	 * <br>
	 * The units for the parameters must be consistent with each other. For example, if maximum velocity is in feet/second, 
	 * then maximum acceleration must be in feet/second^2, base width must be in feet, and the units for waypoints also in feet.<br>
	 * <br>
	 * Note this process can take up to half a second, depending on the number of segments.
	 * 
	 * @param waypoints The waypoints the path has to travel through
	 * @param maxVelocity The maximum velocity of the robot
	 * @param maxAcceleration The maximum acceleration of the robot
	 * @param maxJerk The maximum jerk (change in acceleration) of the robot
	 * @param baseWidth The width of the base plate of the robot (distance between left side wheels and right side wheels)
	 * @param alpha Path smoothness constant. A higher alpha makes for smoother turns, but longer distance for the robot to travel
	 * @param segmentCount How many segments the path is split into. A higher count makes the path more precise, but requires more time to generate
	 * @param surpressExceptions If set to true, an exception will <b>not</b> be thrown if the path is impossible
	 */
	public TankDriveTrajectory(Waypoint[] waypoints, double maxVelocity, double maxAcceleration, double maxJerk, double baseWidth, double alpha, int segmentCount, boolean surpressExceptions) {
		this(waypoints, maxVelocity, maxAcceleration, maxAcceleration, maxJerk, baseWidth, alpha, segmentCount, surpressExceptions);
	}
	/**
	 * Generates a trajectory based on a number of parameters.<br>
	 * <br>
	 * The units for the parameters must be consistent with each other. For example, if maximum velocity is in feet/second, 
	 * then maximum acceleration must be in feet/second^2, base width must be in feet, and the units for waypoints also in feet.<br>
	 * <br>
	 * Note this process can take up to half a second, depending on the number of segments.
	 * 
	 * @param waypoints The waypoints the path has to travel through
	 * @param maxVelocity The maximum velocity of the robot
	 * @param maxAcceleration The maximum acceleration of the robot
	 * @param maxDeceleration The maximum deceleration of the robot
	 * @param maxJerk The maximum jerk (change in acceleration) of the robot
	 * @param baseWidth The width of the base plate of the robot (distance between left side wheels and right side wheels)
	 * @param alpha Path smoothness constant. A higher alpha makes for smoother turns, but longer distance for the robot to travel
	 * @param segmentCount How many segments the path is split into. A higher count makes the path more precise, but requires more time to generate
	 */
	public TankDriveTrajectory(Waypoint[] waypoints, double maxVelocity, double maxAcceleration, double maxDeceleration, double maxJerk, double baseWidth, double alpha, int segmentCount) {
		this(waypoints, maxVelocity, maxAcceleration, maxDeceleration, maxJerk, baseWidth, alpha, segmentCount, false);
	}
	/**
	 * Generates a trajectory based on a number of parameters.<br>
	 * <br>
	 * The units for the parameters must be consistent with each other. For example, if maximum velocity is in feet/second, 
	 * then maximum acceleration must be in feet/second^2, base width must be in feet, and the units for waypoints also in feet.<br>
	 * <br>
	 * Note this process can take up to half a second, depending on the number of segments.
	 * 
	 * @param waypoints The waypoints the path has to travel through
	 * @param maxVel The maximum velocity of the robot
	 * @param maxAccel The maximum acceleration of the robot
	 * @param maxDecel The maximum deceleration of the robot
	 * @param maxJerk The maximum jerk (change in acceleration) of the robot
	 * @param baseWidth The width of the base plate of the robot (distance between left side wheels and right side wheels)
	 * @param alpha Path smoothness constant. A higher alpha makes for smoother turns, but longer distance for the robot to travel
	 * @param segmentCount How many segments the path is split into. A higher count makes the path more precise, but requires more time to generate
	 * @param surpressExceptions If set to true, an exception will <b>not</b> be thrown if the path is impossible
	 */
	public TankDriveTrajectory(Waypoint[] waypoints, double maxVel, double maxAccel, double maxDecel, double maxJerk, double baseWidth, double alpha, int segmentCount, boolean surpressExceptions) {
		
		//Generate the path
		path = new BezierPath(waypoints, alpha);
		path.setBaseRadius(baseWidth / 2);
		double t_delta = 1.0 / segmentCount;
		double t_delta2 = t_delta / 2;
		
		TankDrivePathSegment[] segments = new TankDrivePathSegment[segmentCount];
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
			double curvature = MathUtils.curvature(xDeriv, xSecondDeriv, yDeriv, ySecondDeriv);
			
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
			
			//Use the kinematic equation for constant jerk
			//First calculate the coefficients
			double la = maxJerk / 6, lb = leftMoments[i - 1].getAcceleration() / 2, lc = leftMoments[i - 1].getVelocity(), ld = -distDiffLeft;
			double ra = maxJerk / 6, rb = rightMoments[i - 1].getAcceleration() / 2, rc = rightMoments[i - 1].getVelocity(), rd = -distDiffRight;
			//Solve for the time, which is the (only) real root of this cubic polynomial
			double lTime = MathUtils.realCubicRoot(la, lb, lc, ld);
			double rTime = MathUtils.realCubicRoot(ra, rb, rc, rd);
			//Solve for the maximum reachable velocity
			double leftMaxVel = leftMoments[i - 1].getVelocity() + leftMoments[i - 1].getAcceleration() * lTime + 0.5 * maxJerk * Math.pow(lTime, 2);
			double rightMaxVel = rightMoments[i - 1].getVelocity() + rightMoments[i - 1].getAcceleration() * rTime + 0.5 * maxJerk * Math.pow(rTime, 2);
			//Solve for the maximum reachable acceleration
			double leftMaxAccel = leftMoments[i - 1].getAcceleration() + maxJerk * lTime;
			double rightMaxAccel = rightMoments[i - 1].getAcceleration() + maxJerk * rTime;
			
			double leftVel, rightVel;
			double leftAccel, rightAccel;
			
			//Check if our maximum reachable velocity or acceleration is greater than our limit
			//Because the calculation of the max velocity from the segment already takes into account the physical max,
			//we don't need to check for it
			//If our maximum reachable velocity is greater than our limit, then do not accelerate or increase acceleration
			if(leftMaxVel > segments[i - 1].getLeftMaxVelocity()) {
				//Constrain velocity so we don't exceed the max
				leftVel = segments[i - 1].getLeftMaxVelocity();
				//Since acceleration does not increase, we can set the left acceleration to the acceleration from the last moment
				leftAccel = leftMoments[i - 1].getAcceleration();
				//Make the jerk from the last moment 0 so acceleration does not increase
				leftMoments[i - 1].setJerk(0);
				//Even though this will make us exceed the limit, the backwards pass will fix it
			}
			//If our maximum reachable acceleration is greater than our limit, but our maximum reachable velocity is not,
			//Then don't increase acceleration but still accelerate
			else if(leftMaxAccel > maxAccel) {
				//Since acceleration does not increase, we can set the left acceleration to the acceleration from the last moment
				leftAccel = leftMoments[i - 1].getAcceleration();
				//Make the jerk from the last moment 0 so acceleration does not increase
				leftMoments[i - 1].setJerk(0);
				//Calculate the velocity we would reach with just our acceleration
				//This value is guaranteed to be smaller than leftMaxVel
				//Use the kinematic equation for constant acceleration
				leftVel = Math.sqrt(Math.pow(leftMoments[i - 1].getVelocity(), 2) + 2 * maxAccel * distDiffLeft);
			}
			//Otherwise just accelerate fully and set jerk to maximum
			else {
				//Set the velocity to our maximum reachable velocity
				leftVel = leftMaxVel;
				//Set the acceleration to our maximum reachable acceleration
				leftAccel = leftMaxAccel;
				//Set the jerk of the last moment to the maximum jerk
				leftMoments[i - 1].setJerk(maxJerk);
			}
			
			
			//Do the same thing for the right
			if(rightMaxVel > segments[i - 1].getRightMaxVelocity()) {
				//Constrain velocity so we don't exceed the max
				rightVel = segments[i - 1].getRightMaxVelocity();
				//Since acceleration does not increase, we can set the right acceleration to the acceleration from the last moment
				rightAccel = rightMoments[i - 1].getAcceleration();
				//Make the jerk from the last moment 0 so acceleration does not increase
				rightMoments[i - 1].setJerk(0);
				//Even though this will make us exceed the limit, the backwards pass will fix it
			}
			//If our maximum reachable acceleration is greater than our limit, but our maximum reachable velocity is not,
			//Then don't increase acceleration but still accelerate
			else if(rightMaxAccel > maxAccel) {
				//Since acceleration does not increase, we can set the right acceleration to the acceleration from the last moment
				rightAccel = rightMoments[i - 1].getAcceleration();
				//Make the jerk from the last moment 0 so acceleration does not increase
				rightMoments[i - 1].setJerk(0);
				//Calculate the velocity we would reach with just our acceleration
				//This value is guaranteed to be smaller than rightMaxVel
				//Use the kinematic equation for constant acceleration
				rightVel = Math.sqrt(Math.pow(rightMoments[i - 1].getVelocity(), 2) + 2 * maxAccel * distDiffRight);
			}
			//Otherwise just accelerate fully and set jerk to maximum
			else {
				//Set the velocity to our maximum reachable velocity
				rightVel = rightMaxVel;
				//Set the acceleration to our maximum reachable acceleration
				rightAccel = rightMaxAccel;
				//Set the jerk of the last moment to the maximum jerk
				rightMoments[i - 1].setJerk(maxJerk);
			}
			
			//Create our Moment objects with the desired distance, velocity, acceleration and a default jerk of 0
			//which will be changed later if necessary.
			leftMoments[i] = new Moment(currentDists[0], leftVel, leftAccel, 0);
			rightMoments[i] = new Moment(currentDists[1], rightVel, rightAccel, 0);
		}
		
		//Prepare for backwards pass
		leftMoments[segments.length].setVelocity(0);
		leftMoments[segments.length].setAcceleration(0);
		leftMoments[segments.length].setJerk(0);
		rightMoments[segments.length].setVelocity(0);
		rightMoments[segments.length].setAcceleration(0);
		rightMoments[segments.length].setJerk(0);
		/*//Backwards pass
		for(int i = segments.length - 1; i >= 0; i --) {
			//Since we already have our desired distances generated, just retrieve them from the Moment objects
			double distDiffLeft = leftMoments[i + 1].getDistance() - leftMoments[i].getDistance();
			double distDiffRight = rightMoments[i + 1].getDistance() - rightMoments[i].getDistance();
			
			//Use the kinematic equation for constant jerk
			//First calculate the coefficients
			double la = maxJerk / 6, lb = leftMoments[i + 1].getAcceleration() / 2, lc = leftMoments[i + 1].getVelocity(), ld = -distDiffLeft;
			double ra = maxJerk / 6, rb = rightMoments[i + 1].getAcceleration() / 2, rc = rightMoments[i + 1].getVelocity(), rd = -distDiffRight;
			//Solve for the time, which is the (only) real root of this cubic polynomial
			double lTime = MathUtils.realCubicRoot(la, lb, lc, ld);
			double rTime = MathUtils.realCubicRoot(ra, rb, rc, rd);
			//Solve for the maximum reachable velocity
			double leftMaxVel = leftMoments[i + 1].getVelocity() + leftMoments[i + 1].getAcceleration() * lTime + 0.5 * maxJerk * Math.pow(lTime, 2);
			double rightMaxVel = rightMoments[i + 1].getVelocity() + rightMoments[i + 1].getAcceleration() * rTime + 0.5 * maxJerk * Math.pow(rTime, 2);
			//Solve for the maximum reachable acceleration
			double leftMaxAccel = leftMoments[i + 1].getAcceleration() + maxJerk * lTime;
			double rightMaxAccel = rightMoments[i + 1].getAcceleration() + maxJerk * rTime;
			
			double leftVel, rightVel;
			double leftAccel, rightAccel;
			
			//Compare our maximum reachable velocity to the velocity generated by the forwards pass
			if(leftMaxVel > rightMoments[i].getVelocity()) {
				//If it's higher, don't do anything, as the forwards pass should have already settled things
				leftVel = leftMoments[i].getVelocity();
				leftAccel = leftMoments[i].getAcceleration();
			}
			else if(leftMaxAccel > maxDecel) {
				
			}
			else {
				//If it's lower, then correct it, and set this moment to decelerate
				leftVel = leftMax;
				leftMoments[i].setAcceleration(-maxDecel);
			}
			leftMoments[i].setVelocity(leftVel);
			leftMoments[i].setAcceleration(leftAccel);
			//Do the same for the right
			if(rightMax > rightMoments[i].getVelocity()) {
				rightVel = rightMoments[i].getVelocity();
			}
			else {
				rightVel = rightMax;
				rightMoments[i].setAcceleration(-maxDecel);
			}
			rightMoments[i].setVelocity(rightVel);
		}*/
		
		//Now that we have the desired distances, velocities and accelerations, we need to assign each moment a time
		for(int i = 1; i < leftMoments.length; i ++) {
			double distDiffLeft = leftMoments[i].getDistance() - leftMoments[i - 1].getDistance();
			double distDiffRight = rightMoments[i].getDistance() - rightMoments[i - 1].getDistance();
			double leftVel = leftMoments[i - 1].getVelocity();
			double rightVel = rightMoments[i - 1].getVelocity();
			double leftAccel = leftMoments[i - 1].getAcceleration();
			double rightAccel = rightMoments[i - 1].getAcceleration();
			double leftJerk = leftMoments[i - 1].getJerk();
			double rightJerk = rightMoments[i - 1].getJerk();
			
			//Calculate coefficients
			double la = leftJerk / 6, lb = leftAccel / 2, lc = leftVel, ld = -distDiffLeft;
			double ra = rightJerk / 6, rb = rightAccel / 2, rc = rightVel, rd = -distDiffRight;
			//Solve the cubic polynomials to get the time
			double dtLeft = MathUtils.realCubicRoot(la, lb, lc, ld);
			double dtRight = MathUtils.realCubicRoot(ra, rb, rc, rd);
			
			//A result of NaN or Infinity indicates that our path is impossible with the current configuration.
			if(Double.isNaN(dtLeft) || Double.isInfinite(dtLeft)) {
				System.out.printf("*LEFT* Accel: %.10f, Velo: %.10f, Dist: %.10f, Jerk: %.10f, Iteration: %d\n", leftAccel, leftVel, distDiffLeft, leftJerk, i);
				System.out.printf("*LEFT* Equation: %.7fx^3 + %.7fx^2 + %.7fx + %.7f\n", la, lb, lc, ld);
				System.out.println("*LEFT* Discriminant: " + MathUtils.cubicDiscriminant(la, lb, lc, ld));
				if(!surpressExceptions)
					throw new TrajectoryGenerationException("Path is impossible");
				dtLeft = 0;
			}
			if(Double.isNaN(dtRight) || Double.isInfinite(dtRight)) {
				System.out.printf("*RIGHT* Accel: %.10f, Velo: %.10f, Dist: %.10f, Jerk: %.10f, Iteration: %d\n", rightAccel, rightVel, distDiffRight, rightJerk, i);
				System.out.printf("*RIGHT* Equation: %.7fx^3 + %.7fx^2 + %.7fx + %.7f\n", ra, rb, rc, rd);
				System.out.println("*RIGHT* Discriminant: " + MathUtils.cubicDiscriminant(ra, rb, rc, rd));
				if(!surpressExceptions)
					throw new TrajectoryGenerationException("Path is impossible");
				dtRight = 0;
			}
			
			//Add the time differences to the accumulated time of the last moment to get the time of this moment
			leftMoments[i].setTime(leftMoments[i - 1].getTime() + dtLeft);
			rightMoments[i].setTime(rightMoments[i - 1].getTime() + dtRight);
			
			leftMoments[i].lock();
			rightMoments[i].lock();
		}
	}
	//Creates a trajectory with moments that are already generated
	protected TankDriveTrajectory(Moment[] lMoments, Moment[] rMoments) {
		leftMoments = lMoments;
		rightMoments = rMoments;
	}
	protected TankDriveTrajectory(Moment[] lMoments, Moment[] rMoments, BezierPath path) {
		leftMoments = lMoments;
		rightMoments = rMoments;
		this.path = path;
	}
	
	/**
	 * Retrieves the {@code Moment} object associated with the left side at the specified time.<br>
	 * <br>
	 * This method does not interpolate between two {@code Moment}s and thus gives rough results.
	 * Only use if necessary.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the left side at time t
	 */
	public Moment getLeftRaw(double t) {
		//Do binary search to find the closest approximation
		int start = 0;
		int end = leftMoments.length - 1;
		int mid;
		
		//If t is greater than the entire length in time of the left side, return the last Moment
		if(t >= leftMoments[leftMoments.length - 1].getTime())
			return leftMoments[leftMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			if(leftMoments[mid].getTime() == t)
				return leftMoments[mid];
			//If we reached the end, return the end
			if(mid == leftMoments.length - 1)
				return leftMoments[mid];
			//If t is sandwiched between 2 existing times, the return the closest one
			if(leftMoments[mid].getTime() <= t && leftMoments[mid + 1].getTime() >= t) {
				//Use absolute differences to find out the closer Moment
				if(Math.abs(t - leftMoments[mid].getTime()) > Math.abs(t - leftMoments[mid + 1].getTime())) {
					return leftMoments[mid + 1];
				}
				else {
					return leftMoments[mid];
				}
			}
			//Continue the binary search if not found
			if(leftMoments[mid].getTime() < t) {
				start = mid;
				continue;
			}
			else if(leftMoments[mid].getTime() > t) {
				end = mid;
				continue;
			}
		}
	}
	/**
	 * Retrieves the {@code Moment} object associated with the right side at the specified time.<br>
	 * <br>
	 * This method does not interpolate between two {@code Moment}s and thus gives rough results.
	 * Only use if necessary.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the right side at time t
	 */
	public Moment getRightRaw(double t) {
		//For an explanation of the code, refer to getLeft()
		int start = 0;
		int end = rightMoments.length - 1;
		int mid;
		
		if(t >= rightMoments[rightMoments.length - 1].getTime())
			return rightMoments[rightMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			if(rightMoments[mid].getTime() == t)
				return rightMoments[mid];
			
			if(mid == rightMoments.length - 1)
				return rightMoments[mid];
			
			if(rightMoments[mid].getTime() <= t && rightMoments[mid + 1].getTime() >= t) {
				if(Math.abs(t - rightMoments[mid].getTime()) > Math.abs(t - rightMoments[mid + 1].getTime())) {
					return rightMoments[mid + 1];
				}
				else {
					return rightMoments[mid];
				}
			}
			
			if(rightMoments[mid].getTime() < t) {
				start = mid;
				continue;
			}
			else if(rightMoments[mid].getTime() > t) {
				end = mid;
				continue;
			}
		}
	}
	
	/**
	 * Retrieves the {@code Moment} object associated with the left side at the specified time.<br>
	 * <br>
	 * This method retrieves the 2 {@code Moment} objects closest to the specified time, and lerps them to get an
	 * estimate.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the left side at time t
	 */
	public Moment getLeft(double t) {
		//Do binary search to find the closest approximation
		int start = 0;
		int end = leftMoments.length - 1;
		int mid;
		
		//If t is greater than the entire length in time of the left side, return the last Moment
		if(t >= leftMoments[leftMoments.length - 1].getTime())
			return leftMoments[leftMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			double midTime = leftMoments[mid].getTime();
			
			if(midTime == t)
				return leftMoments[mid];
			//If we reached the end, return the end
			if(mid == leftMoments.length - 1)
				return leftMoments[mid];
			
			double nextTime = leftMoments[mid + 1].getTime();
			
			//If t is sandwiched between 2 existing times, the return the closest one
			if(midTime <= t && nextTime >= t) {
				//Get the slopes
				double dt = nextTime - midTime;
				double mAccel = (leftMoments[mid + 1].getAcceleration() - leftMoments[mid].getAcceleration()) / dt;
				double mVel = (leftMoments[mid + 1].getVelocity() - leftMoments[mid].getVelocity()) / dt;
				double mDist = (leftMoments[mid + 1].getDistance() - leftMoments[mid].getDistance()) / dt;
				//Linear approximation
				double t2 = t - midTime;
				return new Moment(mDist * t2 + leftMoments[mid].getDistance(), 
						mVel * t2 + leftMoments[mid].getVelocity(),
						mAccel * t2 + leftMoments[mid].getAcceleration(), t);
			}
			//Continue the binary search if not found
			if(midTime < t) {
				start = mid;
				continue;
			}
			else if(midTime > t) {
				end = mid;
				continue;
			}
		}
	}
	/**
	 * Retrieves the {@code Moment} object associated with the right side at the specified time.<br>
	 * <br>
	 * This method retrieves the 2 {@code Moment} objects closest to the specified time, and lerps them to get an
	 * estimate.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the right side at time t
	 */
	public Moment getRight(double t) {
		//Do binary search to find the closest approximation
		int start = 0;
		int end = rightMoments.length - 1;
		int mid;
		
		//If t is greater than the entire length in time of the left side, return the last Moment
		if(t >= rightMoments[rightMoments.length - 1].getTime())
			return rightMoments[rightMoments.length - 1];
		
		while(true) {
			mid = (start + end) / 2;
			
			double midTime = rightMoments[mid].getTime();
			
			if(midTime == t)
				return rightMoments[mid];
			//If we reached the end, return the end
			if(mid == rightMoments.length - 1)
				return rightMoments[mid];
			
			double nextTime = rightMoments[mid + 1].getTime();
			
			//If t is sandwiched between 2 existing times, the return the closest one
			if(midTime <= t && nextTime >= t) {
				//Get the slopes
				double dt = nextTime - midTime;
				double mAccel = (rightMoments[mid + 1].getAcceleration() - rightMoments[mid].getAcceleration()) / dt;
				double mVel = (rightMoments[mid + 1].getVelocity() - rightMoments[mid].getVelocity()) / dt;
				double mDist = (rightMoments[mid + 1].getDistance() - rightMoments[mid].getDistance()) / dt;
				//Linear approximation
				double t2 = t - midTime;
				return new Moment(mDist * t2 + rightMoments[mid].getDistance(), 
						mVel * t2 + rightMoments[mid].getVelocity(),
						mAccel * t2 + rightMoments[mid].getAcceleration(), t);
			}
			//Continue the binary search if not found
			if(midTime < t) {
				start = mid;
				continue;
			}
			else if(midTime > t) {
				end = mid;
				continue;
			}
		}
	}
	
	/**
	 * Retrieves the rounding limit for the solving of quadratic equations.<br>
	 * <br>
	 * During the last stage of trajectory generation, a time is assigned to every moment.
	 * This is done by using the third kinematic equation and thus requires solving quadratic equations.
	 * As with the quadratic formula, if the quantity {@code b^2-4ac < 0}, the equation will have no real solutions
	 * and will result in {@code NaN}. Sometimes this value dips just below 0, causing a seemingly possible
	 * trajectory to fail. However, a robot in real life would never need such a high degree of precision,
	 * and floating-point math can also introduce rounding errors that accumulate. To fix this issue, when solving
	 * quadratic equations, if the absolute value of {@code b^2-4ac} is less than or equals to the rounding
	 * limit, it will be rounded to 0.<br>
	 * <br>
	 * The default value for the rounding limit is 1.0e-5.
	 * @return The rounding limit
	 */
	public static double getSolverRoundingLimit() {
		return minUnit;
	}
	/**
	 * Sets the rounding limit for the solving of quadratic equations.<br>
	 * <br>
	 * During the last stage of trajectory generation, a time is assigned to every moment.
	 * This is done by using the third kinematic equation and thus requires solving quadratic equations.
	 * As with the quadratic formula, if the discriminant is negative, the equation will have no real solutions
	 * and will result in {@code NaN}. Sometimes this value dips just below 0, causing a seemingly possible
	 * trajectory to fail. However, a robot in real life would never need such a high degree of precision,
	 * and floating-point math can also introduce rounding errors that accumulate. To fix this issue, when solving
	 * quadratic equations, if the absolute value of the discriminant is less than or equals to the rounding
	 * limit, it will be rounded to 0.<br>
	 * <br>
	 * The default value for the rounding limit is 1.0e-5.
	 * @param min The new rounding limit
	 */
	public static void setSolverRoundingLimit(double min) {
		minUnit = min;
	}
	
	/**
	 * Retrieves the total time needed for the robot to complete this trajectory.
	 * @return The total time required to complete this trajectory
	 */
	public double totalTime() {
		//Return the length in time of the longer side
		return Math.max(leftMoments[leftMoments.length - 1].getTime(), rightMoments[rightMoments.length - 1].getTime());
	}
	
	/**
	 * Retrieves the underlying {@code BezierPath} used to generate this trajectory.
	 * @return The internal {@code BezierPath}
	 */
	public BezierPath getPath() {
		return path;
	}
	/**
	 * Accesses the internal {@code BezierPath} object and returns the path value at the specified time.<br>
	 * This value should <b>not</b> be used directly for motion planning. It is a path, not a trajectory.
	 * @param t A positive real number ranging from 0 to 1
	 * @return The X and Y values at the specified time on the path
	 */
	public Vec2D pathAt(double t) {
		return path.at(t);
	}
	
	/**
	 * Returns the left-right mirror image of this trajectory. Every left turn will now become a right turn.<br>
	 * <br>
	 * Internally, this is done by creating a new trajectory with the left and right wheels swapped. No new 
	 * arrays are created, so this method is very fast.
	 * @see TankDriveTrajectory#mirrorFrontBack() mirrorFrontBack()
	 * @return The mirrored trajectory
	 */
	public TankDriveTrajectory mirrorLeftRight() {
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Vec2D refPoint = new Vec2D(old[0]);
		Waypoint[] waypoints = new Waypoint[old.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			//Negate the relative x coordinates and flip the angles
			waypoints[i] = new Waypoint(-refPoint.relative(old[i].asVector()).getX(), old[i].getY(), -old[i].getHeading() + Math.PI);
		}
		
		//Just create a new one with the sides swapped
		return new TankDriveTrajectory(rightMoments, leftMoments, new BezierPath(waypoints, path.getAlpha(), path.getBaseRaidus()));
	}
	/**
	 * Returns the front-back mirror image of this trajectory. Every forwards movement will now become
	 * a backwards movement.<br>
	 * <b>Warning: The new trajectory created does not respect maximum deceleration constraints. If you wish
	 * for the new trajectory to respect maximum deceleration constrains, construct a new trajectory with
	 * maximum acceleration and deceleration swapped, and then call this method.</b><br>
	 * <br>
	 * Internally, this is done by creating a new trajectory in which every {@code Moment}'s position, velocity
	 * and acceleration are negated. This means new arrays are being created and copied and thus this method
	 * could be slow for trajectories with many segments.
	 * 
	 * @see TankDriveTrajectory#mirrorLeftRight() mirrorLeftRight()
	 * @return The mirrored trajectory
	 */
	public TankDriveTrajectory mirrorFrontBack() {
		Moment[] lMoments = new Moment[leftMoments.length];
		Moment[] rMoments = new Moment[rightMoments.length];
		
		for(int i = 0; i < lMoments.length; i ++) {
			//Negate the distances, velocities and accelerations to drive backwards
			//Time, of course, always stays positive.
			lMoments[i] = new Moment(-leftMoments[i].getDistance(), -leftMoments[i].getVelocity(), -leftMoments[i].getAcceleration(), leftMoments[i].getTime());
			rMoments[i] = new Moment(-rightMoments[i].getDistance(), -rightMoments[i].getVelocity(), -rightMoments[i].getAcceleration(), rightMoments[i].getTime());
			
			lMoments[i].lock();
			rMoments[i].lock();
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Vec2D refPoint = new Vec2D(old[0]);
		Waypoint[] waypoints = new Waypoint[old.length];
		
		for(int i = 0; i < waypoints.length; i ++) {
			//Negate the relative y coordinates and the headings and keep the x coordinates
			waypoints[i] = new Waypoint(old[i].getX(), -refPoint.relative(old[i].asVector()).getY(), -old[i].getHeading());
		}
		
		BezierPath newPath = new BezierPath(waypoints, path.getAlpha(), path.getBaseRaidus());
		newPath.setDrivingBackwards(true);
		
		return new TankDriveTrajectory(lMoments, rMoments, newPath);
	}
	/**
	 * Returns the reverse of this trajectory. Not to be confused with {@link #retrace()}.
	 * The new trajectory starts at the end of this trajectory and ends at the beginning of it. 
	 * However because the direction is not reversed, the new trajectory does not retrace this trajectory.
	 * Use {@link #retrace()} instead.<br>
	 * <b>Warning: The new trajectory created does not respect maximum deceleration constraints. If you wish
	 * for the new trajectory to respect maximum deceleration constrains, construct a new trajectory with
	 * maximum acceleration and deceleration swapped, and then call this method.</b><br>
	 * <br>
	 * Internally, this is done by creating a new trajectory in which the movements are reversed, that is, the
	 * last {@code Moment}s become the first {@code Moment}s, with the distances adjusted. This means new arrays are being created and copied and thus this method
	 * could be slow for trajectories with many segments.
	 * @see #retrace()
	 * @return The reverse of this trajectory
	 */
	public TankDriveTrajectory reverse() {
		Moment[] lMoments = new Moment[leftMoments.length];
		Moment[] rMoments = new Moment[rightMoments.length];
		
		Moment lLast = leftMoments[leftMoments.length - 1];
		Moment rLast = rightMoments[rightMoments.length - 1];
		
		for(int i = 0; i < lMoments.length; i ++) {
			Moment lm = leftMoments[leftMoments.length - 1 - i];
			Moment rm = rightMoments[rightMoments.length - 1 - i];
			
			//The velocities stay the same
			//Distance and time have to be adjusted since the last Moment has to have distance 0 and time 0
			//Accelerations are negated because if time is reversed, acceleration becomes deceleration
			lMoments[i] = new Moment(lLast.getDistance() - lm.getDistance(), lm.getVelocity(), -lm.getAcceleration(), lLast.getTime() - lm.getTime());
			rMoments[i] = new Moment(rLast.getDistance() - rm.getDistance(), rm.getVelocity(), -rm.getAcceleration(), rLast.getTime() - rm.getTime());
			
			lMoments[i].lock();
			rMoments[i].lock();
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Waypoint[] waypoints = new Waypoint[old.length];
		for(int i = 0; i < old.length; i ++) {
			//New path is just the same as the old path, but with the order of the waypoints reversed,
			//and headings flipped
			waypoints[old.length - 1 - i] = new Waypoint(old[i].getX(), old[i].getY(), (old[i].getHeading() + Math.PI) % (2 * Math.PI));
		}
		return new TankDriveTrajectory(lMoments, rMoments, new BezierPath(waypoints, path.getAlpha(), path.getBaseRaidus()));
	}
	/**
	 * Returns the trajectory that, when driven, would retrace this trajectory and return the robot to its
	 * original position. Not to be confused with {@link #reverse()}.<br>
	 * <b>Warning: The new trajectory created does not respect maximum deceleration constraints. If you wish
	 * for the new trajectory to respect maximum deceleration constrains, construct a new trajectory with
	 * maximum acceleration and deceleration swapped, and then call this method.</b><br>
	 * <br>
	 * Calling this method is essentially equivalent to calling {@link #reverse()} and {@code #mirrorFrontBack()}
	 * in order, but with optimization. 
	 * @see #reverse()
	 * @return The trajectory that retraces this one
	 */
	public TankDriveTrajectory retrace() {
		Moment[] lMoments = new Moment[leftMoments.length];
		Moment[] rMoments = new Moment[rightMoments.length];
		
		Moment lLast = leftMoments[leftMoments.length - 1];
		Moment rLast = rightMoments[rightMoments.length - 1];
		
		for(int i = 0; i < lMoments.length; i ++) {
			Moment lm = leftMoments[leftMoments.length - 1 - i];
			Moment rm = rightMoments[rightMoments.length - 1 - i];

			//A combination of reverse() and mirrorFrontBack()
			lMoments[i] = new Moment(-(lLast.getDistance() - lm.getDistance()), -lm.getVelocity(), lm.getAcceleration(), lLast.getTime() - lm.getTime());
			rMoments[i] = new Moment(-(rLast.getDistance() - rm.getDistance()), -rm.getVelocity(), rm.getAcceleration(), rLast.getTime() - rm.getTime());
			
			lMoments[i].lock();
			rMoments[i].lock();
		}
		
		//Create new path
		Waypoint[] old = path.getWaypoints();
		Waypoint[] waypoints = new Waypoint[old.length];
		for(int i = 0; i < old.length; i ++) {
			//New path is just the same as the old path, but with the order of the waypoints reversed,
			//and headings flipped
			waypoints[old.length - 1 - i] = new Waypoint(old[i].getX(), old[i].getY(), (old[i].getHeading() + Math.PI) % (2 * Math.PI));
		}
		BezierPath newPath = new BezierPath(waypoints, path.getAlpha(), path.getBaseRaidus());
		newPath.setDrivingBackwards(true);
	
		//Note that even though the final path looks exactly the same, the order of the waypoints is actually
		//the opposite.
		return new TankDriveTrajectory(lMoments, rMoments, newPath);
	}
}
