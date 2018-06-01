package robot.pathfinder.examples;

import javax.swing.JFrame;

import robot.pathfinder.TankDriveTrajectory;
import robot.pathfinder.Waypoint;
import robot.pathfinder.tools.Grapher;

/**
 * This example program demonstrates the use of {@link robot.pathfinder.TankDriveTrajectory#retrace()}
 * to create a backwards trajectory.
 * @author Tyler Tian
 *
 */
public class RetraceDemo {

	public static void main(String[] args) {
		//Specify waypoints
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-5, 10, 3 * Math.PI / 4),
				new Waypoint(-20, 20, Math.PI / 2),
		};
		//Create trajectories
		TankDriveTrajectory original = new TankDriveTrajectory(waypoints, 5, 4, 2, 20, 5000);
		//Call the retrace() method to create a new trajectory that retraces the orignal one
		TankDriveTrajectory retraced = original.retrace();
		
		//Graph the trajectories and paths
		JFrame originalPath = Grapher.graphPath(original.getPath(), 0.005);
		JFrame retracedPath = Grapher.graphPath(retraced.getPath(), 0.005);
		JFrame originalTrajectory = Grapher.graphTrajectory(original, 0.01);
		JFrame retracedTrajectory = Grapher.graphTrajectory(retraced, 0.01);
		
		//Give the windows proper titles
		originalPath.setTitle("Original Path");
		retracedPath.setTitle("Retraced Path");
		originalTrajectory.setTitle("Original Trajectory");
		retracedTrajectory.setTitle("Retraced Trajectory");
		
		//Display them
		originalPath.setVisible(true);
		retracedPath.setVisible(true);
		originalTrajectory.setVisible(true);
		retracedTrajectory.setVisible(true);
	}

}
