

import java.awt.Color;
import java.awt.Dimension;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.WaypointEx;
import robot.pathfinder.core.path.JNIPath;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.core.trajectory.TrajectoryGenerator;
import robot.pathfinder.math.Vec2D;
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
		JNIPath path = new JNIPath(new Waypoint[] {
            new Waypoint(0, 0, Math.PI / 2),
            new Waypoint(0, 30, Math.PI / 2),
        }, 10, PathType.QUINTIC_HERMITE);
        path._at(0);

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
		// for(double t = 0; t <= 1 && i < elemCount; t += 0.1) {
		// 	// Collect data
		// 	Vec2D v = path._at(t);
		// 	x[i] = v.getX();
		// 	y[i] = v.getY();
		
		// 	if(graphWheels) {
		// 		var v2 = path._wheelsAt(t);
		// 		leftX[i] = v2.getElem1().getX();
		// 		leftY[i] = v2.getElem1().getY();
		// 		rightX[i] = v2.getElem2().getX();
		// 		rightY[i] = v2.getElem2().getY();
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
		// Vec2D v = path._at(1);
		// x[x.length - 1] = v.getX();
		// y[y.length - 1] = v.getY();
		// if(graphWheels) {
		// 	var v2 = path._wheelsAt(1);
		// 	leftX[leftX.length - 1] = v2.getElem1().getX();
		// 	leftY[leftY.length - 1] = v2.getElem1().getY();
		// 	rightX[rightY.length - 1] = v2.getElem2().getX();
		// 	rightY[rightY.length - 1] = v2.getElem2().getY();
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

		// 	// Create the window that will hold the graph
		// 	JFrame frame = new JFrame("Path Graph");
		// 	frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		// 	frame.setContentPane(plot);
		// 	frame.setResizable(false);
		// 	// Find the maximum size of the window
		// 	Rectangle bounds = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
		// 	// The window has to be square, so take the smaller one of the width and height
		// 	int size = Math.min(bounds.width, bounds.height);
        //     frame.setSize(new Dimension(size, size));
        //     frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        //     frame.setVisible(true);
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
        

        path.free();
        System.out.println("I AM ALIVE!");
	}
}
