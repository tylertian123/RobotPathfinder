package com.arctos6135.robotpathfinder.tools;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;
import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;

import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.Path;
import com.arctos6135.robotpathfinder.core.trajectory.BasicMoment;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveMoment;
import com.arctos6135.robotpathfinder.follower.Followable;
import com.arctos6135.robotpathfinder.math.Vec2D;
import com.arctos6135.robotpathfinder.motionprofile.MotionProfile;
import com.arctos6135.robotpathfinder.util.Pair;

import org.math.plot.Plot2DPanel;

/**
 * A class that provides convenient methods for graphing paths and trajectories.
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public final class Grapher {
	// Private constructor
	private Grapher() {
	}

	// These fields are static instead of local because lambdas using them are used
	private static JFrame frame;
	private static double maxX, maxY, minX, minY;

	/**
	 * Graphs a {@link Followable Followable&lt;BasicMoment&gt;} in a
	 * {@link JFrame}. The heading will not be graphed.
	 * <p>
	 * In addition to graphing, this method also sets the {@link JFrame}'s default
	 * close operation to be {@link WindowConstants#DISPOSE_ON_CLOSE}. Note that
	 * this method does not show the window; {@link JFrame#setVisible(boolean)
	 * setVisible()} needs to be called explicitly in order to show the window.
	 * </p>
	 * 
	 * @param trajectory The trajectory to graph
	 * @param dt         The time increment between samples
	 * @return The graphed trajectory in a {@link JFrame}
	 */
	public static JFrame graphBasicFollowable(Followable<BasicMoment> trajectory, double dt) {
		return graphBasicFollowable(trajectory, dt, false);
	}

	/**
	 * Graphs a {@link Followable Followable&lt;BasicMoment&gt;} in a
	 * {@link JFrame}.
	 * <p>
	 * In addition to graphing, this method also sets the {@link JFrame}'s default
	 * close operation to be {@link WindowConstants#DISPOSE_ON_CLOSE}. Note that
	 * this method does not show the window; {@link JFrame#setVisible(boolean)
	 * setVisible()} needs to be called explicitly in order to show the window.
	 * </p>
	 * 
	 * @param trajectory   The trajectory to graph
	 * @param dt           The time increment between samples
	 * @param graphHeading Whether or not to graph the heading
	 * @return The graphed trajectory in a {@link JFrame}
	 */
	public static JFrame graphBasicFollowable(Followable<BasicMoment> trajectory, double dt, boolean graphHeading) {
		int elemCount = (int) Math.ceil(trajectory.totalTime() / dt);

		// Create arrays to store data
		double[] time = new double[elemCount];
		double[] pos = new double[elemCount];
		double[] vel = new double[elemCount];
		double[] acl = new double[elemCount];
		double[] heading = graphHeading ? new double[elemCount] : null;

		int i = 0;
		for (double t = 0; t <= trajectory.totalTime() && i < elemCount; t += dt) {
			// Collect data
			time[i] = t;
			BasicMoment m = trajectory.get(t);

			pos[i] = m.getPosition();
			vel[i] = m.getVelocity();
			acl[i] = m.getAcceleration();
			if (graphHeading) {
				heading[i] = m.getFacingRelative();
			}

			i++;
		}

		Runnable r = () -> {
			// Create plot
			Plot2DPanel plot = new Plot2DPanel();
			plot.setLegendOrientation("EAST");
			// Add graphs
			plot.addLinePlot("Position", time, pos);
			plot.addLinePlot("Velocity", time, vel);
			plot.addLinePlot("Acceleration", time, acl);
			if (graphHeading) {
				plot.addLinePlot("Heading", time, heading);
			}

			// Create window that holds the graph
			frame = new JFrame("Trajectory Graph");
			frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
			frame.setContentPane(plot);
			// Maximize
			frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		};
		// Since everything has to be done on the EDT, check if we are on it already
		if (SwingUtilities.isEventDispatchThread()) {
			r.run();
		} else {
			// If not then invoke on EDT
			try {
				SwingUtilities.invokeAndWait(r);
			} catch (InvocationTargetException | InterruptedException e) {
				e.printStackTrace();
			}
		}
		return frame;
	}

	/**
	 * Graphs a {@link Followable Followable&lt;TankDriveMoment&gt;} in a
	 * {@link JFrame}. The heading will not be graphed.
	 * <p>
	 * In addition to graphing, this method also sets the {@link JFrame}'s default
	 * close operation to be {@link WindowConstants#DISPOSE_ON_CLOSE}. Note that
	 * this method does not show the window; {@link JFrame#setVisible(boolean)
	 * setVisible()} needs to be called explicitly in order to show the window.
	 * </p>
	 * 
	 * @param trajectory The trajectory to graph
	 * @param dt         The time increment between samples
	 * @return The graphed trajectory in a {@link JFrame}
	 */
	public static JFrame graphTankDriveFollowable(Followable<TankDriveMoment> trajectory, double dt) {
		return graphTankDriveFollowable(trajectory, dt, false);
	}

	/**
	 * Graphs a {@link Followable Followable&lt;TankDriveMoment&gt;} in a
	 * {@link JFrame}.
	 * <p>
	 * In addition to graphing, this method also sets the {@link JFrame}'s default
	 * close operation to be {@link WindowConstants#DISPOSE_ON_CLOSE}. Note that
	 * this method does not show the window; {@link JFrame#setVisible(boolean)
	 * setVisible()} needs to be called explicitly in order to show the window.
	 * </p>
	 * 
	 * @param trajectory   The trajectory to graph
	 * @param dt           The time increment between samples
	 * @param graphHeading Whether or not to graph the relative facing.
	 * @return The graphed trajectory in a {@link JFrame}
	 */
	public static JFrame graphTankDriveFollowable(Followable<TankDriveMoment> trajectory, double dt,
			boolean graphHeading) {
		int elemCount = (int) Math.ceil(trajectory.totalTime() / dt);

		// Create arrays to store data
		double[] time = new double[elemCount];
		double[] lPos = new double[elemCount];
		double[] lVel = new double[elemCount];
		double[] lAcl = new double[elemCount];
		double[] rPos = new double[elemCount];
		double[] rVel = new double[elemCount];
		double[] rAcl = new double[elemCount];
		double[] heading = graphHeading ? new double[elemCount] : null;

		int i = 0;
		for (double t = 0; t <= trajectory.totalTime() && i < elemCount; t += dt) {
			// Collect data
			time[i] = t;
			TankDriveMoment m = trajectory.get(t);

			lPos[i] = m.getLeftPosition();
			lVel[i] = m.getLeftVelocity();
			lAcl[i] = m.getLeftAcceleration();
			rPos[i] = m.getRightPosition();
			rVel[i] = m.getRightVelocity();
			rAcl[i] = m.getRightAcceleration();
			if (graphHeading) {
				heading[i] = m.getFacingRelative();
			}

			i++;
		}

		Runnable r = () -> {
			// Create plot
			Plot2DPanel plot = new Plot2DPanel();
			plot.setLegendOrientation("EAST");
			// Add graphs
			plot.addLinePlot("Left Position", time, lPos);
			plot.addLinePlot("Left Velocity", time, lVel);
			plot.addLinePlot("Left Acceleration", time, lAcl);
			plot.addLinePlot("Right Position", time, rPos);
			plot.addLinePlot("Right Velocity", time, rVel);
			plot.addLinePlot("Right Acceleration", time, rAcl);
			if (graphHeading) {
				plot.addLinePlot("Robot Heading", time, heading);
			}

			// Create window that holds the graph
			frame = new JFrame("Trajectory Graph");
			frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
			frame.setContentPane(plot);
			// Maximize
			frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		};
		if (SwingUtilities.isEventDispatchThread()) {
			r.run();
		} else {
			try {
				SwingUtilities.invokeAndWait(r);
			} catch (InvocationTargetException | InterruptedException e) {
				e.printStackTrace();
			}
		}
		return frame;
	}

	/**
	 * Graphs a {@link Path} in a {@link JFrame}.
	 * <p>
	 * In addition to graphing, this method also sets the {@link JFrame}'s default
	 * close operation to be {@link WindowConstants#DISPOSE_ON_CLOSE}, disables
	 * resizing and resizes it to show the entire path. It is not recommended to
	 * change the size of the frame, as this might mess up the axis scales. Note
	 * that this method does not show the window; the user needs to call
	 * {@link JFrame#setVisible(boolean) setVisible()} explicitly in order to show
	 * the window.
	 * </p>
	 * 
	 * @param path The path to graph
	 * @param dt   The time increment between samples
	 * @return The graphed path in a {@link JFrame}
	 */
	public static JFrame graphPath(Path path, double dt) {
		// Divide and round up to get the number of samples
		// Add 1 for the last sample (see below)
		int elemCount = (int) Math.ceil(1.0 / dt) + 1;

		boolean graphWheels = path.getBaseRadius() != 0 && !Double.isNaN(path.getBaseRadius());

		// Create arrays to hold samples
		double[] x = new double[elemCount];
		double[] y = new double[elemCount];
		double[] leftX = new double[elemCount];
		double[] leftY = new double[elemCount];
		double[] rightX = new double[elemCount];
		double[] rightY = new double[elemCount];

		// These values are used later to determine the size of the graph
		minX = Double.MAX_VALUE;
		minY = Double.MAX_VALUE;
		maxX = -Double.MAX_VALUE;
		maxY = -Double.MAX_VALUE;

		Waypoint[] waypoints = path.getWaypoints();
		double[][] xy = new double[waypoints.length][2];
		for (int j = 0; j < waypoints.length; j++) {
			xy[j][0] = waypoints[j].getX();
			xy[j][1] = waypoints[j].getY();

			minX = Math.min(minX, waypoints[j].getX());
			minY = Math.min(minY, waypoints[j].getY());
			maxX = Math.max(maxX, waypoints[j].getX());
			maxY = Math.max(maxY, waypoints[j].getY());
		}

		int i = 0;
		for (double t = 0; t <= 1 && i < elemCount; t += dt) {
			// Collect data
			Vec2D v = path.at(t);
			x[i] = v.getX();
			y[i] = v.getY();

			if (graphWheels) {
				Pair<Vec2D, Vec2D> v2 = path.wheelsAt(t);
				leftX[i] = v2.getFirst().getX();
				leftY[i] = v2.getFirst().getY();
				rightX[i] = v2.getSecond().getX();
				rightY[i] = v2.getSecond().getY();
			}

			// Update min and max x and y values
			minX = Math.min(minX, v.getX());
			minY = Math.min(minY, v.getY());
			maxX = Math.max(maxX, v.getX());
			maxY = Math.max(maxY, v.getY());

			i++;
		}

		// Collect another sample at t = 1
		// This makes sure the graphed path connects all the waypoints
		Vec2D v = path.at(1);
		x[x.length - 1] = v.getX();
		y[y.length - 1] = v.getY();
		if (graphWheels) {
			Pair<Vec2D, Vec2D> v2 = path.wheelsAt(1);
			leftX[leftX.length - 1] = v2.getFirst().getX();
			leftY[leftY.length - 1] = v2.getFirst().getY();
			rightX[rightY.length - 1] = v2.getSecond().getX();
			rightY[rightY.length - 1] = v2.getSecond().getY();
		}
		minX = Math.min(minX, v.getX());
		minY = Math.min(minY, v.getY());
		maxX = Math.max(maxX, v.getX());
		maxY = Math.max(maxY, v.getY());

		Runnable r = () -> {
			// Graph path
			Plot2DPanel plot = new Plot2DPanel();
			plot.setLegendOrientation("EAST");
			plot.addLinePlot("Robot Center", x, y);

			if (graphWheels) {
				plot.addLinePlot("Left Wheel", leftX, leftY);
				plot.addLinePlot("Right Wheel", rightX, rightY);
			}
			
			plot.addScatterPlot("Waypoints", Color.BLACK, xy);

			// Take the longer of the two differences
			// We want the smallest square that can fit the whole graph
			double len = Math.max(maxX - minX, maxY - minY);

			double xOffset = (len - (maxX - minX)) / 2;
			double yOffset = (len - (maxY - minY)) / 2;

			// Set bounds to make the x and y scales the same
			// Make the square a bit bigger than the graph to leave a margin
			plot.setFixedBounds(0, minX - 2 * path.getBaseRadius() - xOffset,
					minX + len + 2 * path.getBaseRadius() - xOffset);
			plot.setFixedBounds(1, minY - 2 * path.getBaseRadius() - yOffset,
					minY + len + 2 * path.getBaseRadius() - yOffset);

			// Create the window that will hold the graph
			frame = new JFrame("Path Graph");
			frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
			frame.setContentPane(plot);
			frame.setResizable(false);
			// Find the maximum size of the window
			Rectangle bounds = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
			// The window has to be square, so take the smaller one of the width and height
			int size = Math.min(bounds.width, bounds.height);
			frame.setSize(new Dimension(size, size));
		};

		if (SwingUtilities.isEventDispatchThread()) {
			r.run();
		} else {
			try {
				SwingUtilities.invokeAndWait(r);
			} catch (InvocationTargetException | InterruptedException e) {
				e.printStackTrace();
			}
		}
		return frame;
	}

	/**
	 * Graphs a {@link MotionProfile} in a {@link JFrame}.
	 * <p>
	 * In addition to graphing, this method also sets the {@link JFrame}'s default
	 * close operation to be {@link WindowConstants#DISPOSE_ON_CLOSE}. Note that
	 * this method does not show the window; {@link JFrame#setVisible(boolean)
	 * setVisible()} needs to be called explicitly in order to show the window.
	 * </p>
	 * 
	 * @param p  The motion profile to graph
	 * @param dt The time increment between samples
	 * @return The graphed motion profile in a {@link JFrame}
	 */
	public static JFrame graphMotionProfile(MotionProfile p, double dt) {
		int elemCount = (int) Math.ceil(p.totalTime() / dt);

		double[] time = new double[elemCount];
		double[] pos = new double[elemCount];
		double[] vel = new double[elemCount];
		double[] acl = new double[elemCount];

		int i = 0;
		for (double t = 0; t <= p.totalTime() && i < elemCount; t += dt) {
			// Collect data
			time[i] = t;
			pos[i] = p.position(t);
			vel[i] = p.velocity(t);
			acl[i] = p.acceleration(t);

			i++;
		}

		Runnable r = () -> {
			// Create plot
			Plot2DPanel plot = new Plot2DPanel();
			plot.setLegendOrientation("EAST");
			// Add graphs
			plot.addLinePlot("Position", time, pos);
			plot.addLinePlot("Velocity", time, vel);
			plot.addLinePlot("Acceleration", time, acl);

			// Create window that holds the graph
			frame = new JFrame("Trajectory Graph");
			frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
			frame.setContentPane(plot);
			// Maximize
			frame.setExtendedState(JFrame.MAXIMIZED_BOTH);
		};
		if (SwingUtilities.isEventDispatchThread()) {
			r.run();
		} else {
			try {
				SwingUtilities.invokeAndWait(r);
			} catch (InvocationTargetException | InterruptedException e) {
				e.printStackTrace();
			}
		}
		return frame;
	}
}
