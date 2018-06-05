package robot.pathfinder;

import robot.pathfinder.math.MathUtils;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.Moment;

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
	 * @param robotSpecs A {@link robot.pathfinder.RobotSpecs RobotSpecs} object containing the specifications of the robot
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
	 * @param robotSpecs A {@link robot.pathfinder.RobotSpecs RobotSpecs} object containing the specifications of the robot	 
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
			
			//Use the kinematic equation for constant jerk
			//First calculate the coefficients
			double la = maxJerk / 6, lb = leftMoments[i - 1].getAcceleration() / 2, lc = leftMoments[i - 1].getVelocity(), ld = -distDiffLeft;
			double ra = maxJerk / 6, rb = rightMoments[i - 1].getAcceleration() / 2, rc = rightMoments[i - 1].getVelocity(), rd = -distDiffRight;
			//Solve for the time, which is the (only) real root of this cubic polynomial
			double lTime = MathUtils.realCubicRoot(la, lb, lc, ld);
			double rTIme = MathUtils.realCubicRoot(ra, rb, rc, rd);
			
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
			//Since this is a backwards pass, we are now decelerating, so use maxDecel instead of maxAccel
			double leftMax = Math.sqrt(Math.pow(leftMoments[i + 1].getVelocity(), 2) + 2 * maxDecel * leftDistDiff);
			double rightMax = Math.sqrt(Math.pow(rightMoments[i + 1].getVelocity(), 2) + 2 * maxDecel * rightDistDiff);
			double leftVel, rightVel;
			
			//Compare our maximum reachable velocity to the velocity generated by the forwards pass
			if(leftMax > leftMoments[i].getVelocity()) {
				//If it's higher, don't do anything, as the forwards pass should have already settled things
				leftVel = leftMoments[i].getVelocity();
			}
			else {
				//If it's lower, then correct it, and set this moment to decelerate
				leftVel = leftMax;
				leftMoments[i].setAcceleration(-maxDecel);
			}
			leftMoments[i].setVelocity(leftVel);
			//Do the same for the right
			if(rightMax > rightMoments[i].getVelocity()) {
				rightVel = rightMoments[i].getVelocity();
			}
			else {
				rightVel = rightMax;
				rightMoments[i].setAcceleration(-maxDecel);
			}
			rightMoments[i].setVelocity(rightVel);
		}
		
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
			double dtLeft = MathUtils.findPositiveQuadraticRoot(leftAccel / 2, leftVel, -distDiffLeft, minUnit);
			double dtRight = MathUtils.findPositiveQuadraticRoot(rightAccel / 2, rightVel, -distDiffRight, minUnit);
			//A result of NaN or Infinity indicates that our path is impossible with the current configuration.
			if(Double.isNaN(dtLeft) || Double.isInfinite(dtLeft)) {
				System.out.printf("*LEFT* Accel: %.10f, Velo: %.10f, Dist: %.10f, Iteration: %d\n", leftAccel, leftVel, distDiffLeft, i);
				if(!surpressExceptions)
					throw new TrajectoryGenerationException("Path is impossible");
				dtLeft = 0.1;
			}
			if(Double.isNaN(dtRight) || Double.isInfinite(dtRight)) {
				System.out.printf("*RIGHT* Accel: %.10f, Velo: %.10f, Dist: %.10f, Iteration: %d\n", rightAccel, rightVel, distDiffRight, i);
				if(!surpressExceptions)
					throw new TrajectoryGenerationException("Path is impossible");
				dtRight = 0.1;
			}
			
			//Add the time differences to the accumulated time of the last moment to get the time of this moment
			leftMoments[i].setTime(leftMoments[i - 1].getTime() + dtLeft);
			rightMoments[i].setTime(rightMoments[i - 1].getTime() + dtRight);
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
	 * Retrieves the {@code Moment} object associated with the left side at the specified time.
	 * @deprecated Use {@link TankDriveTrajectory#getLeftSmooth(double)} instead.
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
	 * Retrieves the {@code Moment} object associated with the right side at the specified time.
	 * @deprecated Use {@link TankDriveTrajectory#getRightSmooth(double)} instead.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the right side at time t
	 */
	public Moment getRight(double t) {
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
	 * Using this method will yield a smoother result than {@link #getLeft(double)}, but due
	 * to the added steps it will take longer.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the left side at time t
	 */
	public Moment getLeftSmooth(double t) {
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
	 * Using this method will yield a smoother result than {@link #getRight(double)}, but due
	 * to the added steps it will take longer.
	 * @param t A positive real number that ranges from 0 to the return value of {@link #totalTime()}
	 * @return The {@code Moment} object associated with the right side at time t
	 */
	public Moment getRightSmooth(double t) {
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
	 * As with the quadratic formula, if the quantity {@code b^2-4ac < 0}, the equation will have no real solutions
	 * and will result in {@code NaN}. Sometimes this value dips just below 0, causing a seemingly possible
	 * trajectory to fail. However, a robot in real life would never need such a high degree of precision,
	 * and floating-point math can also introduce rounding errors that accumulate. To fix this issue, when solving
	 * quadratic equations, if the absolute value of {@code b^2-4ac} is less than or equals to the rounding
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
