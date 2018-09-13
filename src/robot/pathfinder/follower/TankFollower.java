package robot.pathfinder.follower;

import robot.pathfinder.core.trajectory.TankDriveMoment;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

/**
 * A follower class for tank drive robots and trajectories.
 * <p>
 * Followers are classes that can be given parameters to follow a specific trajectory.
 * They do so using a feedback control system, consisting of 4 gains: velocity feedforward,
 * acceleration feedforward, proportional gain, and derivative gain.
 * </p>
 * @author Tyler Tian
 *
 */
public class TankFollower extends Follower {

	TankDriveTrajectory traj;
	TimestampSource timer;
	DistanceSource lDistSrc, rDistSrc;
	Motor lMotor, rMotor;
	
	//Keep track of the initial timestamp and distance measurements so we don't have to reset
	//Keep track of the error and timestamp of the last iteration to calculate the derivative
	double initTime, lastTime, lLastErr, rLastErr, lInitDist, rInitDist;
	
	boolean running = false;
	
	/**
	 * Constructs a new tank drive follower.
	 * @param traj The trajectory to follow
	 * @param lMotor The left side motor
	 * @param rMotor The right side motor
	 * @param lDistSrc A {@link #robot.pathfinder.follower.Follower.DistanceSource DistanceSource} for the left motor
	 * @param rDistSrc A {@link #robot.pathfinder.follower.Follower.DistanceSource DistanceSource} for the right motor
	 * @param timer A {@link #robot.pathfinder.follower.Follower.TimestampSource TimestampSource} to grab timestamps from
	 * @param kV The velocity feedforward 
	 * @param kA The acceleration feedforward
	 * @param kP The proportional gain
	 * @param kD The derivative gain
	 */
	public TankFollower(TankDriveTrajectory traj, Motor lMotor, Motor rMotor, 
			DistanceSource lDistSrc, DistanceSource rDistSrc, TimestampSource timer,
			double kV, double kA, double kP, double kD) {
		setGains(kV, kA, kP, kD);
		this.traj = traj;
		this.lMotor = lMotor;
		this.rMotor = rMotor;
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
		this.timer = timer;
	}
	
	/**
	 * Sets the trajectory to follow.
	 * @param traj The new trajectory to follow
	 * @throws RuntimeException If the follower is running
	 */
	public void setTrajectory(TankDriveTrajectory traj) {
		if(running) {
			throw new RuntimeException("Trajectory cannot be changed when follower is running");
		}
		this.traj = traj;
	}
	/**
	 * Sets the timestamp source.
	 * @param timer The new timestamp source
	 * @throws RuntimeException If the follower is running
	 */
	public void setTimestampSource(TimestampSource timer) {
		if(running) {
			throw new RuntimeException("Timestamp Source cannot be changed when follower is running");
		}
		this.timer = timer;
	}
	/**
	 * Sets the motors.
	 * @param lMotor The left motor
	 * @param rMotor The right motor
	 * @throws RuntimeException If the follower is running
	 */
	public void setMotors(Motor lMotor, Motor rMotor) {
		if(running) {
			throw new RuntimeException("Motors cannot be changed when follower is running");
		}
		this.lMotor = lMotor;
		this.rMotor = rMotor;
	}
	/**
	 * Sets the distance sources.
	 * @param lDistSrc The left distance source
	 * @param rDistSrc The right distance source
	 * @throws RuntimeException If the follower is running
	 */
	public void setDistanceSources(DistanceSource lDistSrc, DistanceSource rDistSrc) {
		if(running) {
			throw new RuntimeException("Distance Sources cannot be changed when follower is running");
		}
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
	}
	
	/**
	 * Tests whether the follower is running. The follower is considered to be "running" if {@link initialize()}
	 * has been called, the trajectory did not end, and {@link stop()} has not been called.
	 * @return Whether the follower is currently running
	 */
	public boolean isRunning() {
		return running;
	}
	/**
	 * {@inheritDoc}<br>
	 * <br>
	 * If the follower is currently running, this method will do nothing.
	 */
	public void initialize() {
		if(running) {
			return;
		}
		lInitDist = lDistSrc.getDistance();
		rInitDist = rDistSrc.getDistance();
		initTime = lastTime = timer.getTimestamp();
		
		running = true;
	}
	/**
	 * {@inheritDoc}<br>
	 * <br>
	 * If the follower is not initialized (not running), this method will first call {@link #initialize()}
	 * and then perform one cycle of the control loop.
	 */
	public void run() {
		if(!running) {
			initialize();
		}
		
		//Calculate current t and time difference from last iteration
		double timestamp = timer.getTimestamp();
		double dt = timestamp - lastTime;
		double t = timestamp - initTime;
		if(t > traj.totalTime()) {
			stop();
			return;
		}
		
		TankDriveMoment m = traj.get(t);
		//Calculate left and right errors
		double leftErr = (lDistSrc.getDistance() - lInitDist) - m.getLeftPosition();
		double rightErr = (rDistSrc.getDistance() -lInitDist) - m.getRightPosition();
		//Get the derivative of the errors
		//Subtract away the desired velocity to get the true error
		double leftDeriv = (leftErr - lLastErr) / dt 
    			- m.getLeftVelocity();
    	double rightDeriv = (rightErr - rLastErr) / dt
    			- m.getRightVelocity();
    	//Calculate outputs
    	double leftOutput = kA * m.getLeftAcceleration() + kV * m.getLeftVelocity()
				+ kP * leftErr + kD * leftDeriv;
		double rightOutput = kA * m.getRightAcceleration() + kV * m.getRightVelocity()
				+ kP * rightErr + kD * rightDeriv;
		//Constrain
    	leftOutput = Math.max(-1, Math.min(1, leftOutput));
    	rightOutput = Math.max(-1, Math.min(1, rightOutput));
    	
    	lMotor.set(leftOutput);
    	rMotor.set(rightOutput);
    	
    	lastTime = timer.getTimestamp();
    	lLastErr = leftErr;
    	rLastErr = rightErr;
		
	}
	/**
	 * {@inheritDoc}
	 */
	public void stop() {
		lMotor.set(0);
		rMotor.set(0);
		
		running = false;
	}

}
