import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.Path;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.tools.Grapher;

/**
 * This example program generates a trajectory, graphs it, and displays it in a window.
 * @author Tyler Tian
 *
 */
public class TrajectoryGenerationDemo {

	public static void main(String[] args) {
		// Did you know that this trajectory generation code can be generated by the trajectory visualization tool?
		// Give it a try!
		
		// To create a trajectory, first we need the robot specifications object
		// The hypothetical robot has max speed of 8 ft/sec, max acceleration of 6 ft/sec and a base plate 2 feet wide
		RobotSpecs robotSpecs = new RobotSpecs(8.0, 6.0, 2.0);
		// Now create the trajectory parameters object
		TrajectoryParams params = new TrajectoryParams();
		// Specify the waypoints
		// Note that all angles are in radians!
		params.waypoints = new Waypoint[] {
				// Try changing these and see what happens!
				new Waypoint(0.0, 0.0, Math.PI / 2),
				new Waypoint(-10.0, 14.0, Math.PI / 2),
				new Waypoint(0.0, 25.0, 0.0),
		};
		// The "alpha" value is the turn smoothness constant. For more information, see its documentation.
		// Try tweaking this value and see what happens!
		params.alpha = 20.0;
		// There are also many other trajectory parameters, but we will leave them at the default value for now.

		// Finally, generate the trajectory
		TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);
		// Now that we have the trajectory, graph it using the Grapher utility class, and show it
		// Because we need to free the reference to the path to prevent a memory leak, a variable has to be declared.
		Path path = trajectory.getPath();
		JFrame pathGraph = Grapher.graphPath(path, 0.01);
		path.free();
		JFrame trajectoryGraph = Grapher.graphTrajectory(trajectory, 0.01);
		trajectory.free();
		// Since swing is not thread safe, the windows have to be shown on the EDT
		SwingUtilities.invokeLater(() -> {
			pathGraph.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			trajectoryGraph.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			pathGraph.setVisible(true);
			trajectoryGraph.setVisible(true);	
		});
	}

}
