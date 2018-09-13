package robot.pathfinder.follower;

import robot.pathfinder.core.trajectory.TankDriveMoment;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

public class TankFollower extends Follower {

	TankDriveTrajectory traj;
	TimestampSource timer;
	DistanceSource lDistSrc, rDistSrc;
	Motor lMotor, rMotor;
	
	double initTime, lastTime, lLastErr, rLastErr;
	boolean running = false;
	
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
	
	public void setTrajectory(TankDriveTrajectory traj) {
		if(running) {
			throw new RuntimeException("Trajectory cannot be changed when follower is running");
		}
		this.traj = traj;
	}
	public void setTimestampSource(TimestampSource timer) {
		if(running) {
			throw new RuntimeException("Timestamp Source cannot be changed when follower is running");
		}
		this.timer = timer;
	}
	public void setMotors(Motor lMotor, Motor rMotor) {
		if(running) {
			throw new RuntimeException("Motors cannot be changed when follower is running");
		}
		this.lMotor = lMotor;
		this.rMotor = rMotor;
	}
	public void setDistanceSources(DistanceSource lDistSrc, DistanceSource rDistSrc) {
		if(running) {
			throw new RuntimeException("Distance Sources cannot be changed when follower is running");
		}
		this.lDistSrc = lDistSrc;
		this.rDistSrc = rDistSrc;
	}
	
	public boolean isRunning() {
		return running;
	}
	public void initialize() {
		if(running) {
			return;
		}
		lDistSrc.resetDistance();
		rDistSrc.resetDistance();
		initTime = lastTime = timer.getTimestamp();
		
		running = true;
	}
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
		double leftErr = lDistSrc.getDistance() - m.getLeftPosition();
		double rightErr = rDistSrc.getDistance() - m.getRightPosition();
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
	public void stop() {
		lMotor.set(0);
		rMotor.set(0);
		
		running = false;
	}

}
