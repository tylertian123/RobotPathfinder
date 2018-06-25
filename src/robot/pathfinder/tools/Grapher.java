package robot.pathfinder.tools;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;

import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.core.BasicTrajectory;
import robot.pathfinder.core.BezierPath;
import robot.pathfinder.core.Moment;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.tankdrive.TankDriveTrajectory;

/**
 * A class that provides convenient methods for graphing paths and trajectories.
 * @author Tyler Tian
 *
 */
public class Grapher {
	//Private constructor
	private Grapher() {}
	
	public static JFrame graphTrajectory(BasicTrajectory trajectory, double dt) {
		int elemCount = (int) (trajectory.totalTime() / dt) + 1;
		
		//Create arrays to store data
		double[] time = new double[elemCount];
		double[] pos = new double[elemCount];
		double[] vel = new double[elemCount];
		double[] acl = new double[elemCount];
		
		int i = 0;
		for(double t = 0; t <= trajectory.totalTime(); t += dt) {
			//Collect data
			time[i] = t;
			Moment m = trajectory.get(t);
			
			pos[i] = m.getPosition();
			vel[i] = m.getVelocity();
			acl[i] = m.getAcceleration();
			
			i++;
		}
		
		//Create plot
		Plot2DPanel plot = new Plot2DPanel();
		plot.setLegendOrientation("EAST");
		//Add graphs
		plot.addLinePlot("Position", time, pos);
		plot.addLinePlot("Velocity", time, vel);
		plot.addLinePlot("Acceleration", time, acl);
		
		//Create window that holds the graph
		JFrame frame = new JFrame("Trajectory Graph");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(plot);
		//Maximize
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		return frame;
	}
	
	public static JFrame graphTrajectory(TankDriveTrajectory trajectory, double dt) {
		int elemCount = (int) (trajectory.totalTime() / dt) + 1;
		
		//Create arrays to store data
		double[] time = new double[elemCount];
		double[] lPos = new double[elemCount];
		double[] lVel = new double[elemCount];
		double[] lAcl = new double[elemCount];
		double[] rPos = new double[elemCount];
		double[] rVel = new double[elemCount];
		double[] rAcl = new double[elemCount];
		
		int i = 0;
		for(double t = 0; t <= trajectory.totalTime(); t += dt) {
			//Collect data
			time[i] = t;
			Moment lMoment = trajectory.getLeft(t);
			Moment rMoment = trajectory.getRight(t);
			
			lPos[i] = lMoment.getPosition();
			lVel[i] = lMoment.getVelocity();
			lAcl[i] = lMoment.getAcceleration();
			rPos[i] = rMoment.getPosition();
			rVel[i] = rMoment.getVelocity();
			rAcl[i] = rMoment.getAcceleration();
			
			i++;
		}
		
		//Create plot
		Plot2DPanel plot = new Plot2DPanel();
		plot.setLegendOrientation("EAST");
		//Add graphs
		plot.addLinePlot("Left Position", time, lPos);
		plot.addLinePlot("Left Velocity", time, lVel);
		plot.addLinePlot("Left Acceleration", time, lAcl);
		plot.addLinePlot("Right Position", time, rPos);
		plot.addLinePlot("Right Velocity", time, rVel);
		plot.addLinePlot("Right Acceleration", time, rAcl);
		
		//Create window that holds the graph
		JFrame frame = new JFrame("Trajectory Graph");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(plot);
		//Maximize
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		return frame;
	}
	
	/**
	 * Graphs a {@link BezierPath} in a {@link JFrame} with JMathPlot<br>
	 * <br>
	 * In addition to graphing, this method also sets the {@code JFrame}'s default close operation to be
	 * {@code JFrame.DISPOSE_ON_CLOSE}, disables resizing and resizes it to show the entire path. It is
	 * not recommended to change the size of the frame, as this might mess up the axis scales.<br>
	 * Note that this method does not show the window; the user needs to call {@link JFrame#setVisible(boolean) setVisible()}
	 * explicitly in order to show the window.
	 * @param path The {@link BezierPath} to show
	 * @param dt The time increment between samples (0 to 1)
	 * @return A frame with the graphed path inside
	 */
	public static JFrame graphPath(BezierPath path, double dt) {
		//Divide and round up to get the number of samples
		int elemCount = (int) Math.ceil(1.0 / dt);
		
		//Create arrays to hold samples
		double[] x = new double[elemCount];
		double[] y = new double[elemCount];
		double[] leftX = new double[elemCount];
		double[] leftY = new double[elemCount];
		double[] rightX = new double[elemCount];
		double[] rightY = new double[elemCount];
		
		//These values are used later to determine the size of the graph
		double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE;
		double maxX = Double.MIN_VALUE, maxY = Double.MIN_VALUE;
		
		int i = 0;
		for(double t = 0; t <= 1; t += dt) {
			//Collect data
			Vec2D v = path.at(t);
			Vec2D[] v2 = path.wheelsAt(t);
			
			x[i] = v.getX();
			y[i] = v.getY();
			leftX[i] = v2[0].getX();
			leftY[i] = v2[0].getY();
			rightX[i] = v2[1].getX();
			rightY[i] = v2[1].getY();
			
			//Update min and max x and y values
			minX = Math.min(minX, v.getX());
			minY = Math.min(minY, v.getY());
			maxX = Math.max(maxX, v.getX());
			maxY = Math.max(maxY, v.getY());
			
			i ++;
		}
		
		//Graph path
		Plot2DPanel plot = new Plot2DPanel();
		plot.setLegendOrientation("EAST");
		plot.addLinePlot("Robot Center", x, y);
		plot.addLinePlot("Left Wheel", leftX, leftY);
		plot.addLinePlot("Right Wheel", rightX, rightY);
		
		Waypoint[] waypoints = path.getWaypoints();
		//Fixes a bug with JMathPlot
		double[][] xy = new double[2][waypoints.length > 2 ? waypoints.length : 3];
		for(int j = 0; j < path.getWaypoints().length; j ++) {
			xy[0][j] = waypoints[j].getX();
			xy[1][j] = waypoints[j].getY();
		}
		if(waypoints.length < 3) {
			if(waypoints.length < 2) {
				xy[0][1] = Double.MIN_VALUE;
				xy[1][1] = Double.MIN_VALUE;
			}
			xy[0][2] = Double.MIN_VALUE;
			xy[1][2] = Double.MIN_VALUE;
		}
		plot.addScatterPlot("Waypoints", Color.BLACK, xy);
		

		//Take the longer of the two differences
		//We want the smallest square that can fit the whole graph
		double len = Math.max(maxX - minX, maxY - minY);
		
		double xOffset = (len - (maxX - minX)) / 2;
		double yOffset = (len - (maxY - minY)) / 2;
		
		
		//Set bounds to make the x and y scales the same
		//Make the square a bit bigger than the graph to leave a margin
		plot.setFixedBounds(0, minX - 2 * path.getBaseRaidus() - xOffset, minX + len + 2 * path.getBaseRaidus() - xOffset);
		plot.setFixedBounds(1, minY - 2 * path.getBaseRaidus() - yOffset, minY + len + 2 * path.getBaseRaidus() - yOffset);
		
		//Create the window that will hold the graph
		JFrame frame = new JFrame("Path Graph");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(plot);
		frame.setResizable(false);
		//Find the maximum size of the window
		Rectangle bounds = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
		//The window has to be square, so take the smaller one of the width and height
		int size = Math.min(bounds.width, bounds.height);
		frame.setSize(new Dimension(size, size));
		
		return frame;
	}
	
	public static JFrame graphMoments(Moment[] moments) {
		double[] pos = new double[moments.length];
		double[] vel = new double[moments.length];
		double[] acl = new double[moments.length];
		
		for(int i = 0; i < moments.length; i ++) {
			pos[i] = moments[i].getPosition();
			vel[i] = moments[i].getVelocity();
			acl[i] = moments[i].getAcceleration();
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.setLegendOrientation("EAST");
		plot.addLinePlot("Position", pos);
		plot.addLinePlot("Velocity", vel);
		plot.addLinePlot("Acceleration", acl);
		
		JFrame frame = new JFrame("Moment Graph");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(plot);
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		return frame;
	}
}
