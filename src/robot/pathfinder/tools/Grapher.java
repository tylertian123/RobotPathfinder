package robot.pathfinder.tools;

import java.awt.Dimension;
import java.awt.GraphicsEnvironment;
import java.awt.Rectangle;

import javax.swing.JFrame;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.BezierPath;
import robot.pathfinder.Moment;
import robot.pathfinder.TankDriveTrajectory;
import robot.pathfinder.math.Vec2D;

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
	 * @param trajectory - The {@link TankDriveTrajectory} to graph
	 * @param dt - The time increment between samples
	 * @return A frame with the graphed trajectory inside
	 */
	public static JFrame graphTrajectory(TankDriveTrajectory trajectory, double dt) {
		int elemCount = (int) (trajectory.totalTime() / dt) + 1;
		
		double[] time = new double[elemCount];
		double[] leftPos = new double[elemCount];
		double[] rightPos = new double[elemCount];
		double[] leftVel = new double[elemCount];
		double[] rightVel = new double[elemCount];
		double[] leftAccel = new double[elemCount];
		double[] rightAccel = new double[elemCount];
		
		int i = 0;
		for(double t = 0; t <= trajectory.totalTime(); t += dt) {
			time[i] = t;
			Moment leftMoment = trajectory.getLeftSmooth(t);
			Moment rightMoment = trajectory.getRightSmooth(t);
			
			leftPos[i] = leftMoment.getDistance();
			leftVel[i] = leftMoment.getVelocity();
			leftAccel[i] = leftMoment.getAcceleration();
			rightPos[i] = rightMoment.getDistance();
			rightVel[i] = rightMoment.getVelocity();
			rightAccel[i] = rightMoment.getAcceleration();
			
			i++;
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.setLegendOrientation("EAST");
		plot.addLinePlot("Left Position", time, leftPos);
		plot.addLinePlot("Right Position", time, rightPos);
		plot.addLinePlot("Left Velocity", time, leftVel);
		plot.addLinePlot("Right Velocity", time, rightVel);
		plot.addLinePlot("Left Acceleration", time, leftAccel);
		plot.addLinePlot("Right Acceleration", time, rightAccel);
		
		JFrame frame = new JFrame("Trajectory Graph");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(plot);
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
	 * @param path - The {@link BezierPath} to show
	 * @param dt - The time increment between samples (0 to 1)
	 * @return A frame with the graphed path inside
	 */
	public static JFrame graphPath(BezierPath path, double dt) {
		int elemCount = (int) Math.ceil(1.0 / dt);
		
		double[] x = new double[elemCount];
		double[] y = new double[elemCount];
		double[] leftX = new double[elemCount];
		double[] leftY = new double[elemCount];
		double[] rightX = new double[elemCount];
		double[] rightY = new double[elemCount];
		
		double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE;
		double maxX = Double.MIN_VALUE, maxY = Double.MIN_VALUE;
		
		int i = 0;
		for(double t = 0; t <= 1; t += dt) {
			Vec2D v = path.at(t);
			Vec2D[] v2 = path.wheelsAt(t);
			
			x[i] = v.getX();
			y[i] = v.getY();
			leftX[i] = v2[0].getX();
			leftY[i] = v2[0].getY();
			rightX[i] = v2[1].getX();
			rightY[i] = v2[1].getY();
			
			minX = Math.min(minX, v.getX());
			minY = Math.min(minY, v.getY());
			maxX = Math.max(maxX, v.getX());
			maxY = Math.max(maxY, v.getY());
			
			i ++;
		}
		
		Plot2DPanel plot = new Plot2DPanel();
		plot.setLegendOrientation("EAST");
		plot.addLinePlot("Robot Center", x, y);
		plot.addLinePlot("Left Wheel", leftX, leftY);
		plot.addLinePlot("Right Wheel", rightX, rightY);
		

		double len = Math.max(maxX - minX, maxY - minY);
		
		plot.setFixedBounds(0, minX - path.getBaseRaidus(), minX + len + path.getBaseRaidus());
		plot.setFixedBounds(1, minY - path.getBaseRaidus(), minY + len + path.getBaseRaidus());
		
		JFrame frame = new JFrame("Path Graph");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setContentPane(plot);
		frame.setResizable(false);
		Rectangle bounds = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
		int size = Math.min(bounds.width, bounds.height);
		frame.setSize(new Dimension(size, size));
		
		return frame;
	}
}
