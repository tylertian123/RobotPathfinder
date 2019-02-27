

import javax.swing.JFrame;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.WaypointEx;
import robot.pathfinder.core.path.JNIPath;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.core.trajectory.TrajectoryGenerator;
import robot.pathfinder.tools.Grapher;

public class DebugTests {
    
    public static void test18() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
        TrajectoryParams params = new TrajectoryParams();
        params.waypoints = new Waypoint[] {
                new WaypointEx(0.0, 0.0, Math.PI / 4, 2.0),
                new WaypointEx(25.0, 25.0, Math.PI / 2, 0),
                new WaypointEx(40.0, 25.0, -Math.PI / 2, 2.0),
        };
        params.alpha = 40.0;
        params.segmentCount = 1000;
        params.isTank = true;
        params.pathType = PathType.QUINTIC_HERMITE;
        BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);

        JFrame f1 = Grapher.graphPath(trajectory.getPath(), 0.01);
        f1.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f1.setVisible(true);
        JFrame f2 = Grapher.graphTrajectory(trajectory, 0.01);
        f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f2.setVisible(true);
    }
    
    public static void test19() {
        RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 3.0);

        TankDriveTrajectory traj = TrajectoryGenerator.generateRotationTank(robotSpecs, -Math.PI);

        JFrame f1 = Grapher.graphTrajectory(traj, 0.001, true);
        f1.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        f1.setVisible(true);
    }

	public static void main(String[] args) throws Exception {
		// JNIPath path = new JNIPath(new Waypoint[] {
        //     new Waypoint(0, 0, Math.PI / 2),
        //     new Waypoint(0, 30, Math.PI / 2),
        // }, 10, PathType.QUINTIC_HERMITE);

        // // Divide and round up to get the number of samples
		// // Add 1 for the last sample (see below)
		// int elemCount = (int) Math.ceil(1.0 / 0.01) + 1;
		
		// boolean graphWheels = path.getBaseRadius() != 0;
		
		// // Create arrays to hold samples
		// double[] x = new double[elemCount];
		// double[] y = new double[elemCount];
		// double[] leftX = new double[elemCount];
		// double[] leftY = new double[elemCount];
		// double[] rightX = new double[elemCount];
		// double[] rightY = new double[elemCount];
        
        // double minX, minY, maxX, maxY;
		// // These values are used later to determine the size of the graph
		// minX = Double.MAX_VALUE; minY = Double.MAX_VALUE;
		// maxX = -Double.MAX_VALUE; maxY = -Double.MAX_VALUE;
		
		// int i = 0;
		// for(double t = 0; t <= 1 && i < elemCount; t += dt) {
		// 	// Collect data
		// 	Vec2D v = path._at(t);
		// 	x[i] = v.getX();
		// 	y[i] = v.getY();
			
		// 	if(graphWheels) {
		// 		Vec2D[] v2 = path.wheelsAt(t);
		// 		leftX[i] = v2[0].getX();
		// 		leftY[i] = v2[0].getY();
		// 		rightX[i] = v2[1].getX();
		// 		rightY[i] = v2[1].getY();
		// 	}
			
		// 	// Update min and max x and y values
		// 	minX = Math.min(minX, v.getX());
		// 	minY = Math.min(minY, v.getY());
		// 	maxX = Math.max(maxX, v.getX());
		// 	maxY = Math.max(maxY, v.getY());
			
		// 	i ++;
		// }
		
		// // Collect another sample at t = 1
		// // This makes sure the graphed path connects all the waypoints
		// Vec2D v = path.at(1);
		// x[x.length - 1] = v.getX();
		// y[y.length - 1] = v.getY();
		// if(graphWheels) {
		// 	Vec2D[] v2 = path.wheelsAt(1);
		// 	leftX[leftX.length - 1] = v2[0].getX();
		// 	leftY[leftY.length - 1] = v2[0].getY();
		// 	rightX[rightY.length - 1] = v2[1].getX();
		// 	rightY[rightY.length - 1] = v2[1].getY();
		// }
		// minX = Math.min(minX, v.getX());
		// minY = Math.min(minY, v.getY());
		// maxX = Math.max(maxX, v.getX());
		// maxY = Math.max(maxY, v.getY());
		
		// Runnable r = () -> {
		// 	// Graph path
		// 	Plot2DPanel plot = new Plot2DPanel();
		// 	plot.setLegendOrientation("EAST");
		// 	plot.addLinePlot("Robot Center", x, y);
			
		// 	if(graphWheels) {
		// 		plot.addLinePlot("Left Wheel", leftX, leftY);
		// 		plot.addLinePlot("Right Wheel", rightX, rightY);
		// 	}
			
		// 	Waypoint[] waypoints = path.getWaypoints();
		// 	// Fixes a bug with JMathPlot
		// 	double[][] xy = new double[2][waypoints.length > 2 ? waypoints.length : 3];
		// 	for(int j = 0; j < path.getWaypoints().length; j ++) {
		// 		xy[0][j] = waypoints[j].getX();
		// 		xy[1][j] = waypoints[j].getY();
		// 	}
		// 	if(waypoints.length < 3) {
		// 		if(waypoints.length < 2) {
		// 			xy[0][1] = -Double.MAX_VALUE;
		// 			xy[1][1] = -Double.MAX_VALUE;
		// 		}
		// 		xy[0][2] = -Double.MAX_VALUE;
		// 		xy[1][2] = -Double.MAX_VALUE;
		// 	}
		// 	plot.addScatterPlot("Waypoints", Color.BLACK, xy);
			
	
		// 	// Take the longer of the two differences
		// 	// We want the smallest square that can fit the whole graph
		// 	double len = Math.max(maxX - minX, maxY - minY);
			
		// 	double xOffset = (len - (maxX - minX)) / 2;
		// 	double yOffset = (len - (maxY - minY)) / 2;
			
			
		// 	// Set bounds to make the x and y scales the same
		// 	// Make the square a bit bigger than the graph to leave a margin
		// 	plot.setFixedBounds(0, minX - 2 * path.getBaseRadius() - xOffset, minX + len + 2 * path.getBaseRadius() - xOffset);
		// 	plot.setFixedBounds(1, minY - 2 * path.getBaseRadius() - yOffset, minY + len + 2 * path.getBaseRadius() - yOffset);
			
		// 	// Create the window that will hold the graph
		// 	frame = new JFrame("Path Graph");
		// 	frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		// 	frame.setContentPane(plot);
		// 	frame.setResizable(false);
		// 	// Find the maximum size of the window
		// 	Rectangle bounds = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
		// 	// The window has to be square, so take the smaller one of the width and height
		// 	int size = Math.min(bounds.width, bounds.height);
		// 	frame.setSize(new Dimension(size, size));
		// };
		
		// if(SwingUtilities.isEventDispatchThread()) {
		// 	r.run();
		// }
		// else {
		// 	try {
		// 		SwingUtilities.invokeAndWait(r);
		// 	} 
		// 	catch (InvocationTargetException | InterruptedException e) {
		// 		e.printStackTrace();
		// 	}
		// }

        // path.free();
        // System.out.println("I AM ALIVE!");
	}
}
