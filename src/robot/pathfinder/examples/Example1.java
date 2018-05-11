package robot.pathfinder.examples;

import java.util.ArrayList;

import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.BezierPath;
import robot.pathfinder.Moment;
import robot.pathfinder.TankDriveTrajectory;
import robot.pathfinder.Waypoint;
import robot.pathfinder.math.Vec2D;

/**
 * An example program that generates and graphs a path and trajectory.
 * This was the program that generated the graphs in the project README.
 * @author Tyler
 *
 */
public class Example1 {
	/**
	 * Creates a {@code double} array from an {@code ArrayList} of {@code Double}s.
	 * @param a - An {@code ArrayList} of {@code Double}s
	 * @return A {@code double} array with the elements in the list
	 */
	static double[] primitiveArr(ArrayList<Double> a) {
		Double[] arr = new Double[a.size()];
		a.toArray(arr);
		double[] d = new double[arr.length];
		for(int i = 0; i < arr.length; i ++)
			d[i] = arr[i];
		return d;
	}
	
	public static void main(String[] args) {
		//Create array of waypoints
		Waypoint[] waypoints = new Waypoint[] {
				//X pos, Y pos, direction (radians)
				//Direction follows the unit circle, so 0 is the positive X-axis direction
				//Try changing these and watch what happens!
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-10, 50, 0),
		};
		//Used to determine how long the generation took
		long time = System.currentTimeMillis();
		//Generates a trajectory
		//Parameters: Waypoints, max vel, max accel, base width, alpha (smoothness of turns), number of segments
		//Last parameter suppresses exceptions (allows impossible paths)
		//Try changing these and see what happens!
		TankDriveTrajectory trajectory = new TankDriveTrajectory(waypoints, 5, 3.5, 2, 20, 5000, true);
		System.out.println("Trajectory generation took " + (System.currentTimeMillis() - time) + " milliseconds.");
		
		final double t_delta = 0.001;
		
		//Create a bunch of lists to store the data at each time
		ArrayList<Double> xs = new ArrayList<Double>();
		ArrayList<Double> ys = new ArrayList<Double>();
		ArrayList<Double> lx = new ArrayList<Double>();
		ArrayList<Double> ly = new ArrayList<Double>();
		ArrayList<Double> rx = new ArrayList<Double>();
		ArrayList<Double> ry = new ArrayList<Double>();
		
		ArrayList<Double> lds = new ArrayList<Double>();
		ArrayList<Double> lvs = new ArrayList<Double>();
		ArrayList<Double> las = new ArrayList<Double>();
		ArrayList<Double> rds = new ArrayList<Double>();
		ArrayList<Double> rvs = new ArrayList<Double>();
		ArrayList<Double> ras = new ArrayList<Double>();
		ArrayList<Double> ts = new ArrayList<Double>();
		
		//Retrieve the path (not the trajectory!)
		BezierPath path = trajectory.getPath();
		//Paths start at t=0, and always end at t=1
		for(double t = 0; t <= 1; t += t_delta) {
			//Retrieve the position of the center of the robot
			Vec2D v = path.at(t);
			//Add the x and y of the center to the lists
			xs.add(v.getX());
			ys.add(v.getY());
			//Retrieve the position of the wheels of the robot
			Vec2D[] wheels = path.wheelsAt(t);
			//Add the x and y of the wheels to the lists
			lx.add(wheels[0].getX());
			ly.add(wheels[0].getY());
			rx.add(wheels[1].getX());
			ry.add(wheels[1].getY());
		}
		System.out.println(trajectory.totalTime());
		//Loop from t=0 to the total time to complete this trajectory, with 0.005s increments
		//Notice how here we use the trajectory instead of the path
		for(double t = 0; t <= trajectory.totalTime(); t += 0.005) {
			//Get the desired distance, velocity and acceleration for the sides at the desired time
			//getSideSmooth() linearly interpolates between two points to allow for a smoother trajectory
			//Try changing this to getLeft() and getRight() and see what happens!
			Moment left = trajectory.getLeftSmooth(t);
			Moment right = trajectory.getRightSmooth(t);
			//Add the values to the lists
			lds.add(left.getDistance());
			rds.add(right.getDistance());
			lvs.add(left.getVelocity());
			rvs.add(right.getVelocity());
			las.add(left.getAcceleration());
			ras.add(right.getAcceleration());
			ts.add(t);
		}
		
		//Create the graphs using JMathPlot
		//Its documentation can be found on its repository at https://github.com/yannrichet/jmathplot/
		Plot2DPanel plot = new Plot2DPanel();
		//Here we have to convert our list to a primitive type array, since addLinePlot() only accepts double[]s
		plot.addLinePlot("Bezier", primitiveArr(xs), primitiveArr(ys));
		plot.addLinePlot("Left", primitiveArr(lx), primitiveArr(ly));
		plot.addLinePlot("Right", primitiveArr(rx), primitiveArr(ry));
		//Add legend
		plot.setLegendOrientation("EAST");
		//Create the first JFrame to hold the graph for the path
		JFrame frame = new JFrame("Bezier");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		//Maximize and show
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame.setContentPane(plot);
		frame.setVisible(true);
		
		//Same thing as above
		Plot2DPanel plot2 = new Plot2DPanel();
		//Convert the time to an array and assign a variable to it for reuse
		double[] times = primitiveArr(ts);
		plot2.addLinePlot("Left Position", times, primitiveArr(lds));
		plot2.addLinePlot("Left Velocity", times, primitiveArr(lvs));
		plot2.addLinePlot("Left Acceleration", times, primitiveArr(las));
		plot2.addLinePlot("Right Position", times, primitiveArr(rds));
		plot2.addLinePlot("Right Velocity", times, primitiveArr(rvs));
		plot2.addLinePlot("Right Acceleration", times, primitiveArr(ras));
		plot2.setLegendOrientation("EAST");
		JFrame frame2 = new JFrame("PVA");
		frame2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame2.setExtendedState(JFrame.MAXIMIZED_BOTH);
		frame2.setContentPane(plot2);
		frame2.setVisible(true);
	}
}
