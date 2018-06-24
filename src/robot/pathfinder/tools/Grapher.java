package robot.pathfinder.tools;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;

import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.core.Moment;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.math.Vec2D;
import robot.pathfinder.tankdrive.BezierPath;
import robot.pathfinder.tankdrive.TankDriveTrajectory;

/**
 * A class that provides convenient methods for graphing paths and trajectories.
 * @author Tyler Tian
 *
 */
public class Grapher {
	//Private constructor
	private Grapher() {}
	
	/**
	 * Graphs a {@link TankDriveTrajectory} in a {@link JFrame} with JMathPlot.<br>
	 * <br>
	 * In addition to graphing, this method also sets the {@code JFrame}'s default close operation to be
	 * {@code JFrame.DISPOSE_ON_CLOSE} and maximizes it.<br>
	 * Note that this method does not show the window; the user needs to call {@link JFrame#setVisible(boolean) setVisible()}
	 * explicitly in order to show the window.
	 * @param trajectory The {@link TankDriveTrajectory} to graph
	 * @param dt The time increment between samples
	 * @return A frame with the graphed trajectory inside
	 */
	public static JFrame graphTrajectory(TankDriveTrajectory trajectory, double dt) {
		//Divide and round up to get the number of samples
		int elemCount = (int) Math.ceil(trajectory.totalTime() / dt);
		
		//Create arrays to store data
		double[] time = new double[elemCount];
		double[] leftPos = new double[elemCount];
		double[] rightPos = new double[elemCount];
		double[] leftVel = new double[elemCount];
		double[] rightVel = new double[elemCount];
		double[] leftAccel = new double[elemCount];
		double[] rightAccel = new double[elemCount];
		double[] leftJerk = new double[elemCount];
		double[] rightJerk = new double[elemCount];
		
		int i = 0;
		for(double t = 0; t <= trajectory.totalTime(); t += dt) {
			//Collect data
			time[i] = t;
			Moment leftMoment = trajectory.getLeft(t);
			Moment rightMoment = trajectory.getRight(t);
			
			leftPos[i] = leftMoment.getPosition();
			leftVel[i] = leftMoment.getVelocity();
			leftAccel[i] = leftMoment.getAcceleration();
			leftJerk[i] = leftMoment.getJerk();
			rightPos[i] = rightMoment.getPosition();
			rightVel[i] = rightMoment.getVelocity();
			rightAccel[i] = rightMoment.getAcceleration();
			rightJerk[i] = rightMoment.getJerk();
			
			i++;
		}
		
		//Create plot
		Plot2DPanel plot = new Plot2DPanel();
		plot.setLegendOrientation("EAST");
		//Add graphs
		plot.addLinePlot("Left Position", time, leftPos);
		plot.addLinePlot("Left Velocity", time, leftVel);
		plot.addLinePlot("Left Acceleration", time, leftAccel);
		plot.addLinePlot("Left Jerk", time, leftJerk);
		plot.addLinePlot("Right Position", time, rightPos);
		plot.addLinePlot("Right Velocity", time, rightVel);
		plot.addLinePlot("Right Acceleration", time, rightAccel);
		plot.addLinePlot("Right Jerk", time, rightJerk);
		
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
	
	/**
	 * Graphs the raw {@link Moment} objects of a trajectory. Instead of taking samples based on time,
	 * this method graphs each {@code Moment}'s data.
	 * @param trajectory The trajectory whose data will be graphed
	 * @return A frame with the graphed trajectory inside
	 */
	public static JFrame graphMoments(TankDriveTrajectory trajectory) {
		Moment[] left = trajectory.getMoments()[0];
		Moment[] right = trajectory.getMoments()[1];
		
		double[] lPos = new double[left.length];
		double[] lVel = new double[left.length];
		double[] lAcl = new double[left.length];
		double[] lJrk = new double[right.length];
		double[] rPos = new double[right.length];
		double[] rVel = new double[right.length];
		double[] rAcl = new double[right.length];
		double[] rJrk = new double[right.length];
		
		for(int i = 0; i < left.length; i ++) {
			lPos[i] = left[i].getPosition();
			lVel[i] = left[i].getVelocity();
			lAcl[i] = left[i].getAcceleration();
			lJrk[i] = left[i].getJerk();
			
			/*rPos[i] = right[i].getPosition();
			rVel[i] = right[i].getVelocity();
			rAcl[i] = right[i].getAcceleration();
			rJrk[i] = right[i].getJerk();*/
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.setLegendOrientation("EAST");
		plot.addLinePlot("Left Position", lPos);
		plot.addLinePlot("Left Velocity", lVel);
		plot.addLinePlot("Left Acceleration", lAcl);
		plot.addLinePlot("Left Jerk", lJrk);
		/*plot.addLinePlot("Right Position", rPos);
		plot.addLinePlot("Right Velocity", rVel);
		plot.addLinePlot("Right Acceleration", rAcl);
		plot.addLinePlot("Right Jerk", rJrk);*/
		
		JFrame frame = new JFrame("Moment Graph");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(plot);
		frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		
		return frame;
	}
}
