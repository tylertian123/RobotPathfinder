package robot.pathfinder.tools;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.ListSelectionModel;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.filechooser.FileFilter;
import javax.swing.table.DefaultTableModel;

import org.math.plot.Plot2DPanel;

import robot.pathfinder.BezierPath;
import robot.pathfinder.Moment;
import robot.pathfinder.TrajectoryGenerationException;
import robot.pathfinder.TankDriveTrajectory;
import robot.pathfinder.Waypoint;
import robot.pathfinder.math.Vec2D;

/**
 * A GUI tool build with Swing to help visualize trajectories. 
 * @author Tyler
 *
 */
public class TrajectoryVisualizationTool {

	static JFrame mainFrame;
	static JPanel mainPanel;
	static JPanel buttonsPanel;
	
	static JTextField waypointX = new JTextField();
	static JTextField waypointY = new JTextField();
	static JTextField waypointHeading = new JTextField();
	static JPanel waypointPanel;
	
	static JTextField baseWidth = new JTextField();
	static JTextField alpha = new JTextField();
	static JTextField segments = new JTextField();
	static JTextField maxVelocity = new JTextField();
	static JTextField maxAcceleration = new JTextField();
	static JTextField roundingLimit = new JTextField("1.0e-5");
	static JPanel argumentsPanel;
	
	static ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
	
	static final String[] COLUMN_NAMES = new String[] {
			"X Position",
			"Y Position",
			"Robot Direction (Degrees)"
	};
	
	static double[] primitiveArr(ArrayList<Double> a) {
		Double[] arr = new Double[a.size()];
		a.toArray(arr);
		double[] d = new double[arr.length];
		for(int i = 0; i < arr.length; i ++)
			d[i] = arr[i];
		return d;
	}
	
	static class CSVFilter extends FileFilter {

		@Override
		public boolean accept(File f) {
			if(f.isDirectory())
				return true;
			try {
				String ext = f.getName().substring(f.getName().lastIndexOf("."));
				if(ext.equals(".csv"))
					return true;
				else
					return false;
			}
			catch(StringIndexOutOfBoundsException e) {
				return false;
			}
		}

		@Override
		public String getDescription() {
			return "Comma-Separated Values File (*.csv)";
		}
		
	}
	
	static class WaypointTableModel extends DefaultTableModel {
		/**
		 * 
		 */
		private static final long serialVersionUID = -1823287587692307141L;
		
		public WaypointTableModel(String[] columnNames, int rows) {
			super(columnNames, rows);
		}

		@Override
		public boolean isCellEditable(int row, int col) {
			return false;
		}
	}
	
	public static void main(String[] args) {
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch (ClassNotFoundException | InstantiationException | IllegalAccessException
				| UnsupportedLookAndFeelException e) {
			e.printStackTrace();
		}
		
		waypointX.setPreferredSize(new Dimension(100, 20));
		waypointY.setPreferredSize(new Dimension(100, 20));
		waypointHeading.setPreferredSize(new Dimension(100, 20));
		waypointPanel = new JPanel();
		waypointPanel.setLayout(new BoxLayout(waypointPanel, BoxLayout.PAGE_AXIS));
		waypointPanel.add(new JLabel("Waypoint X"));
		waypointPanel.add(waypointX);
		waypointPanel.add(new JLabel("Waypoint Y"));
		waypointPanel.add(waypointY);
		waypointPanel.add(new JLabel("Robot Direction (Degrees)"));
		waypointPanel.add(waypointHeading);
		
		mainPanel = new JPanel();
		mainPanel.setLayout(new BorderLayout());
		JTable table = new JTable(new WaypointTableModel(COLUMN_NAMES, 0));
		table.setCellSelectionEnabled(false);
		table.setRowSelectionAllowed(true);
		table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		JScrollPane scrollPane = new JScrollPane(table);
		scrollPane.setPreferredSize(new Dimension(620, 300));
		table.setFillsViewportHeight(true);
		mainPanel.add(scrollPane, BorderLayout.PAGE_START);
		
		argumentsPanel = new JPanel();
		Dimension textFieldSize = new Dimension(75, 20);
		maxVelocity.setPreferredSize(textFieldSize);
		maxAcceleration.setPreferredSize(textFieldSize);
		baseWidth.setPreferredSize(textFieldSize);
		alpha.setPreferredSize(textFieldSize);
		segments.setPreferredSize(textFieldSize);
		roundingLimit.setPreferredSize(textFieldSize);
		
		JPanel subPanel1 = new JPanel();
		subPanel1.setLayout(new BoxLayout(subPanel1, BoxLayout.PAGE_AXIS));
		subPanel1.add(new JLabel("Maximum Velocity"));
		subPanel1.add(maxVelocity);
		subPanel1.add(new JLabel("Maximum Acceleration"));
		subPanel1.add(maxAcceleration);
		JPanel subPanel2 = new JPanel();
		subPanel2.setLayout(new BoxLayout(subPanel2, BoxLayout.PAGE_AXIS));
		subPanel2.add(new JLabel("Base Plate Width"));
		subPanel2.add(baseWidth);
		subPanel2.add(new JLabel("Alpha"));
		subPanel2.add(alpha);
		JPanel subPanel3 = new JPanel();
		subPanel3.setLayout(new BoxLayout(subPanel3, BoxLayout.PAGE_AXIS));
		subPanel3.add(new JLabel("Number of Segments"));
		subPanel3.add(segments);
		subPanel3.add(new JLabel("Rounding Limit"));
		subPanel3.add(roundingLimit);
		
		argumentsPanel.add(subPanel1);
		argumentsPanel.add(subPanel2);
		argumentsPanel.add(subPanel3);
		
		mainPanel.add(argumentsPanel, BorderLayout.CENTER);
		
		buttonsPanel = new JPanel();
		
		Dimension buttonSize = new Dimension(120, 30);
		JButton addWaypointButton = new JButton("Add Waypoint");
		addWaypointButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				boolean error = false;
				do {
					int selectedRow = table.getSelectedRow();
					
					int response = JOptionPane.showConfirmDialog(mainFrame, waypointPanel, "New Waypoint...", JOptionPane.OK_CANCEL_OPTION);
					if(response != JOptionPane.OK_OPTION)
						break;
					try {
						double x = Double.parseDouble(waypointX.getText());
						double y = Double.parseDouble(waypointY.getText());
						double heading = Double.parseDouble(waypointHeading.getText());
						
						if(selectedRow == -1)
							waypoints.add(new Waypoint(x, y, Math.toRadians(heading)));
						else 
							waypoints.add(selectedRow, new Waypoint(x, y, Math.toRadians(heading)));
						WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
						if(selectedRow == -1)
							tableModel.addRow(new Object[] { String.valueOf(x), String.valueOf(y), String.valueOf(heading) });
						else
							tableModel.insertRow(selectedRow, new Object[] { String.valueOf(x), String.valueOf(y), String.valueOf(heading) });
						
						error = false;
					}
					catch(NumberFormatException e1) {
						error = true;
						JOptionPane.showMessageDialog(mainFrame, "Error: An invalid token was entered\nin one or more fields.", "Error", JOptionPane.ERROR_MESSAGE);
					}
				} while(error);
				
				waypointX.setText("");
				waypointY.setText("");
				waypointHeading.setText("");
			}
		});
		addWaypointButton.setPreferredSize(buttonSize);
		buttonsPanel.add(addWaypointButton);
		
		JButton deleteWaypointButton = new JButton("Remove Waypoint");
		deleteWaypointButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				int index = table.getSelectedRow();
				if(index == -1) {
					JOptionPane.showMessageDialog(mainFrame, "Error: No waypoint selected.", "Error", JOptionPane.ERROR_MESSAGE);
					return;
				}
				
				waypoints.remove(index);
				WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
				tableModel.removeRow(index);
			}
		});
		deleteWaypointButton.setPreferredSize(buttonSize);
		buttonsPanel.add(deleteWaypointButton);
		
		JButton generateButton = new JButton("Generate");
		generateButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				double maxVel, maxAccel, base, a, minUnit;
				int segmentCount;
				
				try {
					maxVel = Double.parseDouble(maxVelocity.getText());
					maxAccel = Double.parseDouble(maxAcceleration.getText());
					base = Double.parseDouble(baseWidth.getText());
					a = Double.parseDouble(alpha.getText());
					segmentCount = Integer.parseInt(segments.getText());
					minUnit = Double.parseDouble(roundingLimit.getText());
				}
				catch(NumberFormatException e1) {
					JOptionPane.showMessageDialog(mainFrame, "Error: An invalid token was entered\nin one or more fields.", "Error", JOptionPane.ERROR_MESSAGE);
					return;
				}
				
				if(waypoints.size() < 2) {
					JOptionPane.showMessageDialog(mainFrame, "Error: Not enough waypoints.", "Error", JOptionPane.ERROR_MESSAGE);
					return;
				}
				
				Waypoint[] waypointArray = new Waypoint[waypoints.size()];
				for(int i = 0; i < waypointArray.length; i ++) {
					waypointArray[i] = waypoints.get(i);
				}
				
				TankDriveTrajectory trajectory = null;
				try {
					TankDriveTrajectory.setSolverRoundingLimit(minUnit);
					trajectory = new TankDriveTrajectory(waypointArray, maxVel, maxAccel, base, a, segmentCount);
				}
				catch(TrajectoryGenerationException pge) {
					int ret = JOptionPane.showConfirmDialog(mainFrame, "Error: Trajectory generation is impossible with current constraints.\nProceed anyways with only the path?", "Error", JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE);
					if(ret == JOptionPane.NO_OPTION)
						return;
					
					BezierPath path = new BezierPath(waypointArray, a);
					path.setBaseRadius(base / 2);
					
					ArrayList<Double> xPos = new ArrayList<Double>();
					ArrayList<Double> yPos = new ArrayList<Double>();
					ArrayList<Double> leftXPos = new ArrayList<Double>();
					ArrayList<Double> rightXPos = new ArrayList<Double>();
					ArrayList<Double> leftYPos = new ArrayList<Double>();
					ArrayList<Double> rightYPos = new ArrayList<Double>();
					
					for(double t = 0; t <= 1; t += 0.005) {
						Vec2D centerPos = path.at(t);
						Vec2D[] wheelsPos = path.wheelsAt(t);
						
						xPos.add(centerPos.getX());
						yPos.add(centerPos.getY());
						leftXPos.add(wheelsPos[0].getX());
						rightXPos.add(wheelsPos[1].getX());
						leftYPos.add(wheelsPos[0].getY());
						rightYPos.add(wheelsPos[1].getY());
					}
					
					Plot2DPanel pathPlot = new Plot2DPanel();
					pathPlot.setLegendOrientation("EAST");
					pathPlot.addLinePlot("Center Position", primitiveArr(xPos), primitiveArr(yPos));
					pathPlot.addLinePlot("Left Position", primitiveArr(leftXPos), primitiveArr(leftYPos));
					pathPlot.addLinePlot("Right Position", primitiveArr(rightXPos), primitiveArr(rightYPos));
					JFrame pathFrame = new JFrame("Path");
					pathFrame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
					pathFrame.setContentPane(pathPlot);
					pathFrame.setExtendedState(JFrame.MAXIMIZED_BOTH);
					pathFrame.addWindowListener(new WindowAdapter() {
						@Override
						public void windowClosing(WindowEvent e) {
							try {
								UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
							} catch (ClassNotFoundException | InstantiationException | IllegalAccessException
									| UnsupportedLookAndFeelException e1) {
								e1.printStackTrace();
							}
							mainFrame.setVisible(true);
						}
					});
					
					try {
						UIManager.setLookAndFeel(UIManager.getCrossPlatformLookAndFeelClassName());
					} catch (ClassNotFoundException | InstantiationException | IllegalAccessException
							| UnsupportedLookAndFeelException e1) {
						e1.printStackTrace();
					}
					mainFrame.setVisible(false);
					pathFrame.setVisible(true);
					return;
				}
				
				ArrayList<Double> xPos = new ArrayList<Double>();
				ArrayList<Double> yPos = new ArrayList<Double>();
				ArrayList<Double> leftXPos = new ArrayList<Double>();
				ArrayList<Double> rightXPos = new ArrayList<Double>();
				ArrayList<Double> leftYPos = new ArrayList<Double>();
				ArrayList<Double> rightYPos = new ArrayList<Double>();
				
				ArrayList<Double> time = new ArrayList<Double>();
				ArrayList<Double> leftPosition = new ArrayList<Double>();
				ArrayList<Double> rightPosition = new ArrayList<Double>();
				ArrayList<Double> leftVelocity = new ArrayList<Double>();
				ArrayList<Double> rightVelocity = new ArrayList<Double>();
				ArrayList<Double> leftAcceleration = new ArrayList<Double>();
				ArrayList<Double> rightAcceleration = new ArrayList<Double>();
				
				BezierPath path = trajectory.getPath();
				for(double t = 0; t <= 1; t += 0.005) {
					Vec2D centerPos = path.at(t);
					Vec2D[] wheelsPos = path.wheelsAt(t);
					
					xPos.add(centerPos.getX());
					yPos.add(centerPos.getY());
					leftXPos.add(wheelsPos[0].getX());
					rightXPos.add(wheelsPos[1].getX());
					leftYPos.add(wheelsPos[0].getY());
					rightYPos.add(wheelsPos[1].getY());
				}
				for(double t = 0; t <= trajectory.totalTime(); t += 0.010) {
					time.add(t);
					Moment left = trajectory.getLeftSmooth(t);
					Moment right = trajectory.getRightSmooth(t);
					
					leftPosition.add(left.getDistance());
					leftVelocity.add(left.getVelocity());
					leftAcceleration.add(left.getAcceleration());
					rightPosition.add(right.getDistance());
					rightVelocity.add(right.getVelocity());
					rightAcceleration.add(right.getAcceleration());
				}
				
				Plot2DPanel pathPlot = new Plot2DPanel();
				pathPlot.setLegendOrientation("EAST");
				pathPlot.addLinePlot("Center Position", primitiveArr(xPos), primitiveArr(yPos));
				pathPlot.addLinePlot("Left Position", primitiveArr(leftXPos), primitiveArr(leftYPos));
				pathPlot.addLinePlot("Right Position", primitiveArr(rightXPos), primitiveArr(rightYPos));
				
				Plot2DPanel movementPlot = new Plot2DPanel();
				movementPlot.setLegendOrientation("EAST");
				double[] timeArr = primitiveArr(time);
				movementPlot.addLinePlot("Left Position", timeArr, primitiveArr(leftPosition));
				movementPlot.addLinePlot("Left Velocity", timeArr, primitiveArr(leftVelocity));
				movementPlot.addLinePlot("Left Acceleration", timeArr, primitiveArr(leftAcceleration));
				movementPlot.addLinePlot("Right Position", timeArr, primitiveArr(rightPosition));
				movementPlot.addLinePlot("Right Velocity", timeArr, primitiveArr(rightVelocity));
				movementPlot.addLinePlot("Right Acceleration", timeArr, primitiveArr(rightAcceleration));
				
				JFrame pathFrame = new JFrame("Path");
				JFrame movementFrame = new JFrame("Movement");
				pathFrame.setContentPane(pathPlot);
				movementFrame.setContentPane(movementPlot);
				pathFrame.setExtendedState(JFrame.MAXIMIZED_BOTH);
				movementFrame.setExtendedState(JFrame.MAXIMIZED_BOTH);
				
				WindowAdapter closeHook = new WindowAdapter() {
					@Override
					public void windowClosing(WindowEvent e) {
						pathFrame.setVisible(false);
						movementFrame.setVisible(false);
						pathFrame.dispose();
						movementFrame.dispose();
						try {
							UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
						} catch (ClassNotFoundException | InstantiationException | IllegalAccessException
								| UnsupportedLookAndFeelException e1) {
							e1.printStackTrace();
						}
						mainFrame.setVisible(true);
					}
				};
				pathFrame.addWindowListener(closeHook);
				movementFrame.addWindowListener(closeHook);
				
				try {
					UIManager.setLookAndFeel(UIManager.getCrossPlatformLookAndFeelClassName());
				} catch (ClassNotFoundException | InstantiationException | IllegalAccessException
						| UnsupportedLookAndFeelException e1) {
					e1.printStackTrace();
				}
				mainFrame.setVisible(false);
				movementFrame.setVisible(true);
				pathFrame.setVisible(true);
			}
		});
		generateButton.setPreferredSize(buttonSize);
		buttonsPanel.add(generateButton);
		
		JButton saveButton = new JButton("Save");
		saveButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				if(maxVelocity.getText().equals("") || maxAcceleration.getText().equals("") 
						|| baseWidth.getText().equals("") || alpha.getText().equals("")
						|| segments.getText().equals("")) {
					JOptionPane.showMessageDialog(mainFrame, "Error: Please fill in max velocity, max acceleration,\nbase width, alpha and segment count before saving.", "Error", JOptionPane.ERROR_MESSAGE);
					return;
				}
				
				double maxVel, maxAccel, base, a, minUnit;
				int segmentCount;
				
				try {
					maxVel = Double.parseDouble(maxVelocity.getText());
					maxAccel = Double.parseDouble(maxAcceleration.getText());
					base = Double.parseDouble(baseWidth.getText());
					a = Double.parseDouble(alpha.getText());
					segmentCount = Integer.parseInt(segments.getText());
					minUnit = Double.parseDouble(roundingLimit.getText());
				}
				catch(NumberFormatException e1) {
					JOptionPane.showMessageDialog(mainFrame, "Error: An invalid token was entered\nin one or more fields.", "Error", JOptionPane.ERROR_MESSAGE);
					return;
				}
				
				JFileChooser fc = new JFileChooser();
				fc.setDialogTitle("Save As...");
				fc.setAcceptAllFileFilterUsed(false);
				fc.setFileFilter(new CSVFilter());
				fc.setFileSelectionMode(JFileChooser.FILES_ONLY);
				
				int ret = fc.showSaveDialog(mainFrame);
				if(ret == JFileChooser.APPROVE_OPTION) {
					String path = fc.getSelectedFile().getAbsolutePath();
					if(!path.endsWith(".csv"))
						path += ".csv";
					
					try(BufferedWriter out = new BufferedWriter(new FileWriter(path))) {
						out.write(maxVel + "," + maxAccel + "," + base + "," + a + "," + segmentCount + "," + minUnit + "\n");
						
						WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
						for(int row = 0; row < tableModel.getRowCount(); row ++) {
							String x = (String) (tableModel.getValueAt(row, 0));
							String y = (String) (tableModel.getValueAt(row, 1));
							String heading = (String) (tableModel.getValueAt(row, 2));
							
							out.write(x + "," + y + "," + heading + "\n");
						}
						JOptionPane.showMessageDialog(mainFrame, "Data saved successfully.", "Success", JOptionPane.INFORMATION_MESSAGE);
					}
					catch (IOException e1) {
						e1.printStackTrace();
					}
				}
			}
		});
		saveButton.setPreferredSize(buttonSize);
		buttonsPanel.add(saveButton);
		
		JButton loadButton = new JButton("Load");
		loadButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				JFileChooser fc = new JFileChooser();
				fc.setDialogTitle("Load File...");
				fc.setAcceptAllFileFilterUsed(false);
				fc.setFileFilter(new CSVFilter());
				fc.setFileSelectionMode(JFileChooser.FILES_ONLY);
				
				int ret = fc.showSaveDialog(mainFrame);
				if(ret == JFileChooser.APPROVE_OPTION) {
					try(BufferedReader in = new BufferedReader(new FileReader(fc.getSelectedFile()))) {
						String[] parameters = in.readLine().split(",");
						if(parameters.length < 6) {
							JOptionPane.showMessageDialog(mainFrame, "Error: The file format is invalid.", "Error", JOptionPane.ERROR_MESSAGE);
							return;
						}
						maxVelocity.setText(parameters[0]);
						maxAcceleration.setText(parameters[1]);
						baseWidth.setText(parameters[2]);
						alpha.setText(parameters[3]);
						segments.setText(parameters[4]);
						roundingLimit.setText(parameters[5]);
						
						waypoints.clear();
						WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
						tableModel.setRowCount(0);
						
						String line;
						while((line = in.readLine()) != null && !line.equals("")) {
							String[] point = line.split(",");
							Waypoint w = new Waypoint(Double.parseDouble(point[0]), Double.parseDouble(point[1]), Math.toRadians(Double.parseDouble(point[2])));
							waypoints.add(w);
							tableModel.addRow(point);
						}
					}
					catch (IOException e1) {
						e1.printStackTrace();
					}
					
				}
			}
		});
		loadButton.setPreferredSize(buttonSize);
		buttonsPanel.add(loadButton);
		
		mainPanel.add(buttonsPanel, BorderLayout.PAGE_END);
		
		mainFrame = new JFrame("Path Parameters");
		mainFrame.setContentPane(mainPanel);
		mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mainFrame.pack();
		mainFrame.setVisible(true);
	}

}
