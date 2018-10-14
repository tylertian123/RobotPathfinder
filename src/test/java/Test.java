

import java.lang.reflect.InvocationTargetException;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

import org.math.plot.Plot2DPanel;
import org.math.plot.PlotPanel;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveMoment;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;
import robot.pathfinder.core.trajectory.TrajectoryGenerator;
import robot.pathfinder.tools.Grapher;

public class Test {
	public static void test12() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-10, 25, Math.PI / 2),
				new Waypoint(20, 0, 0),
				//new Waypoint(0, 25, Math.PI / 2),
				//new Waypoint(0, 50, Math.PI / 2),
		};
		
		RobotSpecs specs = new RobotSpecs(5, 3.5, 2);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = waypoints;
		params.alpha = 60;
		params.segmentCount = 5000;
		params.isTank = true;
		params.pathType = PathType.QUINTIC_HERMITE;
		
		BasicTrajectory bt = new BasicTrajectory(specs, params);
		TankDriveTrajectory tt = new TankDriveTrajectory(bt);
		
		JFrame f = Grapher.graphPath(tt.getPath(), 0.01);
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setVisible(true);
		
		JFrame f2 = Grapher.graphTrajectory(tt, 0.01);
		f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f2.setVisible(true);
	}
	
	public static void test13() {
		RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 4);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = new Waypoint[] {
				new Waypoint(0.0, 0.0, Math.PI / 2),
				new Waypoint(-10.0, 25.0, Math.PI / 2),
				new Waypoint(0.0, 75.0, 0.0),
		};
		params.alpha = 60.0;
		params.segmentCount = 1000;
		params.isTank = true;
		params.pathType = PathType.QUINTIC_HERMITE;
		TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);
		//BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);
		
		JFrame f = Grapher.graphTrajectory(trajectory, 0.01);
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setVisible(true);
		JFrame f2 = Grapher.graphPath(trajectory.getPath(), 0.01);
		f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f2.setVisible(true);
	}
	
	public static void test14() {
		Grapher.graphTrajectory(TrajectoryGenerator.generateStraightTank(new RobotSpecs(100, 80, 23), -100), 0.1).setVisible(true);
	}
	
	public static void test15() throws InvocationTargetException, InterruptedException {
		RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 4);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = new Waypoint[] {
				new Waypoint(0.0, 0.0, Math.PI / 2),
				new Waypoint(-10.0, 25.0, Math.PI / 2),
				new Waypoint(-10.0, 50.0, Math.PI / 2),
				//new Waypoint(0.0, 75.0, 0.0),
				//new Waypoint(0, 0, 0),
				//new Waypoint(20, 0, 0),
		};
		params.alpha = 50.0;
		params.segmentCount = 5000;
		params.roundingLimit = 1.0E-6;
		params.isTank = true;
		params.pathType = PathType.QUINTIC_HERMITE;
		TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);
		BasicTrajectory trajectory1 = new BasicTrajectory(robotSpecs, params);
		
		//JFrame f = Grapher.graphTrajectory(trajectory, 0.01);
		//f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		//f.setVisible(true);
		//JFrame f2 = Grapher.graphPath(trajectory.getPath(), 0.01);
		//f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		//f2.setVisible(true);
		
		double dt = 0.01;
		double[] v1 = new double[(int) Math.ceil(trajectory.totalTime() / dt)];
		double[] v2 = new double[(int) Math.ceil(trajectory.totalTime() / dt)];
		double[] time = new double[(int) Math.ceil(trajectory.totalTime() / dt)];
		int i = 1;
		for(double t = dt; t <= trajectory.totalTime(); t += dt) {
			TankDriveMoment m = trajectory.get(t);
			v1[i] = m.getLeftVelocity();
			v2[i] = v2[i - 1] + m.getLeftAcceleration() * dt;
			time[i] = t;
			i ++;
		}
		SwingUtilities.invokeAndWait(() -> {
			Plot2DPanel p2p = new Plot2DPanel();
			p2p.addLinePlot("Generated", time, v1);
			p2p.addLinePlot("Integrated", time, v2);
			p2p.setLegendOrientation(PlotPanel.EAST);
			
			JFrame f = new JFrame();
			f.setContentPane(p2p);
			f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			f.setVisible(true);
			
			JFrame f2 = Grapher.graphTrajectory(trajectory, 0.01);
			f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			f2.setVisible(true);
			
			JFrame f3 = Grapher.graphPath(trajectory.getPath(), 0.01);
			f3.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			f3.setVisible(true);
		});
	}
	
	public static void test16() {
		Waypoint[] waypoints = new Waypoint[] {
				new Waypoint(0, 0, Math.PI / 2),
				new Waypoint(-10, 25, Math.PI / 2),
				new Waypoint(20, 0, 0),
				//new Waypoint(0, 25, Math.PI / 2),
				//new Waypoint(0, 50, Math.PI / 2),
		};
		
		RobotSpecs specs = new RobotSpecs(5, 3.5, 2);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = waypoints;
		params.alpha = 60;
		params.segmentCount = 5000;
		params.isTank = true;
		params.pathType = PathType.QUINTIC_HERMITE;
		
		/*long tm1 = System.currentTimeMillis();
		for(int i = 0; i < 50; i ++) {
			BasicTrajectory bt = new BasicTrajectory(specs, params);
			@SuppressWarnings("unused")
			TankDriveTrajectory tt = new TankDriveTrajectory(bt);
		}
		long tm2 = System.currentTimeMillis();
		System.out.println(tm2 - tm1);*/
		
		long tm1 = System.currentTimeMillis();
		BasicTrajectory bt = new BasicTrajectory(specs, params);
		long tm2 = System.currentTimeMillis();
		TankDriveTrajectory tt = new TankDriveTrajectory(bt);
		long tm3 = System.currentTimeMillis();
		System.out.println(tm2 - tm1);
		System.out.println(tm3 - tm2);
	}
	
	public static void test17() {
		RobotSpecs robotSpecs = new RobotSpecs(5.0, 3.5, 2.0);
		TrajectoryParams params = new TrajectoryParams();
		params.waypoints = new Waypoint[] {
				new Waypoint(0.0, 0.0, Math.PI / 4),
				new Waypoint(25.0, 25.0, Math.PI / 2),
				new Waypoint(40.0, 25.0, -Math.PI / 2),
		};
		params.alpha = 40.0;
		params.segmentCount = 1000;
		params.isTank = true;
		params.pathType = PathType.QUINTIC_HERMITE;
		BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params).retrace();
		
		JFrame f1 = Grapher.graphPath(trajectory.getPath(), 0.01);
		f1.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f1.setVisible(true);
		JFrame f2 = Grapher.graphTrajectory(trajectory, 0.1);
		f2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f2.setVisible(true);
	}
	
	public static void main(String[] args) throws Exception {
		test17();
	}
}
