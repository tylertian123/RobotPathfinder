package robot.pathfinder.core.trajectory;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.path.Path;

/**
 * A class that represents a trajectory (motion profile). A trajectory not only defines the points that a 
 * robot will path through, it also provides information about the velocity, acceleration and direction. Most 
 * importantly, for any given time, it can provide information about the robot's whereabouts and other information. 
 * @see TankDriveTrajectory
 * @author Tyler Tian
 *
 */
public interface Trajectory {
	
	public Moment[] getMoments();
	public Moment get(double t);
	public Path getPath();
	public double totalTime();
	
	public RobotSpecs getRobotSpecs();
	public TrajectoryParams getGenerationParams();
	
	public Trajectory mirrorLeftRight();
	public Trajectory mirrorFrontBack();
	public Trajectory retrace();
}
