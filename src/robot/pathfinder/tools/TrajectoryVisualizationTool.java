package robot.pathfinder.tools;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Toolkit;
import java.awt.datatransfer.StringSelection;
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
import java.util.HashMap;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.KeyStroke;
import javax.swing.ListSelectionModel;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.filechooser.FileFilter;
import javax.swing.table.DefaultTableModel;

import com.sun.glass.events.KeyEvent;

import robot.pathfinder.core.RobotSpecs;
import robot.pathfinder.core.TrajectoryParams;
import robot.pathfinder.core.Waypoint;
import robot.pathfinder.core.path.Path;
import robot.pathfinder.core.path.PathType;
import robot.pathfinder.core.trajectory.BasicTrajectory;
import robot.pathfinder.core.trajectory.TankDriveTrajectory;

/**
 * A GUI tool built with Swing to help visualize trajectories. 
 * @author Tyler Tian
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

	static int pathSamples = 200;
	static int trajSamples = 500;
	static JTextField pathSamplesField = new JTextField(String.valueOf(pathSamples));
	static JTextField trajSamplesField = new JTextField(String.valueOf(trajSamples));
	static JPanel sampleCountPanel;
	
	static JTextField baseWidth = new JTextField("0");
	static JTextField alpha = new JTextField();
	static JTextField segments = new JTextField("1000");
	static JTextField maxVelocity = new JTextField();
	static JTextField maxAcceleration = new JTextField();
	static JPanel argumentsPanel;
	
	static JMenuBar menuBar;
	static JMenu fileMenu;
	
	static ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
	
	static final String QHERMITE = "Q";
	static final String CHERMITE = "C";
	static final String BEZIER = "B";
	JRadioButton quinticHermiteButton, cubicHermiteButton, bezierButton;
	static PathType selectedType = PathType.QUINTIC_HERMITE;
	
	JCheckBox isTank;
	
	static final String[] COLUMN_NAMES = new String[] {
			"X Position",
			"Y Position",
			"Robot Direction (Degrees)"
	};
	
	static HashMap<Double, String> specialAngles = new HashMap<>();
	static {
		specialAngles.put(Math.toRadians(30.0), "Math.PI / 6");
		specialAngles.put(Math.toRadians(45.0), "Math.PI / 4");
		specialAngles.put(Math.toRadians(60.0), "Math.PI / 3");
		specialAngles.put(Math.toRadians(90.0), "Math.PI / 2");
		specialAngles.put(Math.toRadians(120.0), "2 * Math.PI / 3");
		specialAngles.put(Math.toRadians(135.0), "3 * Math.PI / 4");
		specialAngles.put(Math.toRadians(150.0), "5 * Math.PI / 6");
		
		specialAngles.put(Math.toRadians(180.0), "Math.PI");
		
		specialAngles.put(Math.toRadians(-30.0), "-Math.PI / 6");
		specialAngles.put(Math.toRadians(-45.0), "-Math.PI / 4");
		specialAngles.put(Math.toRadians(-60.0), "-Math.PI / 3");
		specialAngles.put(Math.toRadians(-90.0), "-Math.PI / 2");
		specialAngles.put(Math.toRadians(-120.0), "-2 * Math.PI / 3");
		specialAngles.put(Math.toRadians(-135.0), "-3 * Math.PI / 4");
		specialAngles.put(Math.toRadians(-150.0), "-5 * Math.PI / 6");
	}
	
	static double[] primitiveArr(ArrayList<Double> a) {
		Double[] arr = new Double[a.size()];
		a.toArray(arr);
		double[] d = new double[arr.length];
		for(int i = 0; i < arr.length; i ++)
			d[i] = arr[i];
		return d;
	}
	
	static double constrainAngle(double angle) {
		if(angle <= 180.0 && angle > -180.0)
			return angle;
		while(angle > 180.0) {
			angle -= 360.0;
		}
		while(angle <= -180.0) {
			angle += 360.0;
		}
		return angle;
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
	
	public TrajectoryVisualizationTool() {
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
		waypointPanel.setLayout(new BoxLayout(waypointPanel, BoxLayout.Y_AXIS));
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
		
		JPanel subPanel1 = new JPanel();
		subPanel1.setLayout(new BoxLayout(subPanel1, BoxLayout.Y_AXIS));
		subPanel1.add(new JLabel("Maximum Velocity"));
		subPanel1.add(maxVelocity);
		subPanel1.add(new JLabel("Maximum Acceleration"));
		subPanel1.add(maxAcceleration);
		JPanel subPanel2 = new JPanel();
		subPanel2.setLayout(new BoxLayout(subPanel2, BoxLayout.Y_AXIS));
		subPanel2.add(new JLabel("Base Plate Width"));
		subPanel2.add(baseWidth);
		subPanel2.add(new JLabel("Alpha"));
		subPanel2.add(alpha);
		JPanel subPanel3 = new JPanel();
		subPanel3.setLayout(new BoxLayout(subPanel3, BoxLayout.Y_AXIS));
		subPanel3.add(new JLabel("Number of Segments"));
		subPanel3.add(segments);
		subPanel3.add(Box.createVerticalStrut(new JLabel().getFont().getSize()));
		isTank = new JCheckBox("Tank Drive");
		isTank.setSelected(true);
		subPanel3.add(isTank);
		JPanel subPanel4 = new JPanel();
		subPanel4.setLayout(new BoxLayout(subPanel4, BoxLayout.Y_AXIS));
		ActionListener changePathType = e -> {
			switch(e.getActionCommand()) {
			case QHERMITE:
				selectedType = PathType.QUINTIC_HERMITE;
				break;
			case CHERMITE:
				selectedType = PathType.CUBIC_HERMITE;
				break;
			case BEZIER:
				selectedType = PathType.BEZIER;
				break;
			default: System.err.println("Error: Unrecognized path type"); break;
			}
		};
		quinticHermiteButton = new JRadioButton("Quintic Hermite");
		quinticHermiteButton.setActionCommand(QHERMITE);
		quinticHermiteButton.setSelected(true);
		quinticHermiteButton.addActionListener(changePathType);
		cubicHermiteButton = new JRadioButton("Cubic Hermite");
		cubicHermiteButton.setActionCommand(CHERMITE);
		cubicHermiteButton.addActionListener(changePathType);
		bezierButton = new JRadioButton("<html>B&#xe9;zier</html>");
		bezierButton.setActionCommand(BEZIER);
		bezierButton.addActionListener(changePathType);
		ButtonGroup pathTypeSelector = new ButtonGroup();
		pathTypeSelector.add(quinticHermiteButton);
		pathTypeSelector.add(cubicHermiteButton);
		pathTypeSelector.add(bezierButton);
		subPanel4.add(quinticHermiteButton);
		subPanel4.add(cubicHermiteButton);
		subPanel4.add(bezierButton);
		
		argumentsPanel.add(subPanel1);
		argumentsPanel.add(subPanel2);
		argumentsPanel.add(subPanel3);
		argumentsPanel.add(subPanel4);
		
		mainPanel.add(argumentsPanel, BorderLayout.CENTER);
		
		buttonsPanel = new JPanel();
		
		Dimension buttonSize = new Dimension(120, 30);
		JButton addWaypointButton = new JButton("Add Waypoint");
		ActionListener addWaypointAction = e -> {
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
						waypoints.add(new Waypoint(x, y, Math.toRadians(constrainAngle(heading))));
					else 
						waypoints.add(selectedRow, new Waypoint(x, y, Math.toRadians(constrainAngle(heading))));
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
		};
		addWaypointButton.addActionListener(addWaypointAction);
		addWaypointButton.setPreferredSize(buttonSize);
		buttonsPanel.add(addWaypointButton);
		
		JButton editWaypointButton = new JButton("Edit Waypoint");
		ActionListener editWaypointAction = e -> {
			int index = table.getSelectedRow();
			if(index == -1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: No waypoint selected.", "Error", JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			String xStr = (String) tableModel.getValueAt(index, 0);
			String yStr = (String) tableModel.getValueAt(index, 1);
			String headingStr = (String) tableModel.getValueAt(index, 2);
			
			waypointX.setText(xStr);
			waypointY.setText(yStr);
			waypointHeading.setText(headingStr);
			
			boolean error;
			do {
				int response = JOptionPane.showConfirmDialog(mainFrame, waypointPanel, "Edit Waypoint...", JOptionPane.OK_CANCEL_OPTION);
				if(response != JOptionPane.OK_OPTION)
					break;
				
				try {
					double x = Double.parseDouble(waypointX.getText());
					double y = Double.parseDouble(waypointY.getText());
					double heading = Double.parseDouble(waypointHeading.getText());
					
					waypoints.set(index, new Waypoint(x, y, Math.toRadians(constrainAngle(heading))));
					tableModel.setValueAt(String.valueOf(x), index, 0);
					tableModel.setValueAt(String.valueOf(y), index, 1);
					tableModel.setValueAt(String.valueOf(heading), index, 2);
					
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
		};
		editWaypointButton.addActionListener(editWaypointAction);
		editWaypointButton.setPreferredSize(buttonSize);
		buttonsPanel.add(editWaypointButton);
		
		JButton deleteWaypointButton = new JButton("Remove Waypoint");
		ActionListener deleteWaypointAction = e -> {
			int index = table.getSelectedRow();
			if(index == -1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: No waypoint selected.", "Error", JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			waypoints.remove(index);
			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			tableModel.removeRow(index);
		};
		deleteWaypointButton.addActionListener(deleteWaypointAction);
		deleteWaypointButton.setPreferredSize(buttonSize);
		buttonsPanel.add(deleteWaypointButton);
		

		JButton previewButton = new JButton("Preview");
		previewButton.addActionListener(e -> {
			double base, a;
			
			try {
				base = isTank.isSelected() ? Double.parseDouble(baseWidth.getText()) : 0;
				a = Double.parseDouble(alpha.getText());
			}
			catch(NumberFormatException e1) {
				JOptionPane.showMessageDialog(mainFrame, isTank.isSelected() ? "Error: Please enter a valid alpha value and base plate width." : "Error: Please enter a valid alpha value.", "Error", JOptionPane.ERROR_MESSAGE);
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
			
			Path path = Path.constructPath(selectedType, waypointArray, a);
			path.setBaseRadius(base / 2);
			
			JFrame pathFrame = Grapher.graphPath(path, 1.0 / pathSamples);
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
			mainFrame.setVisible(false);
			pathFrame.setVisible(true);
		});
		previewButton.setPreferredSize(buttonSize);
		buttonsPanel.add(previewButton);
		
		JButton generateButton = new JButton("Generate");
		generateButton.addActionListener(e -> {
			double maxVel, maxAccel, base, a;
			int segmentCount;
			
			try {
				maxVel = Double.parseDouble(maxVelocity.getText());
				maxAccel = Double.parseDouble(maxAcceleration.getText());
				base = isTank.isSelected() ? Double.parseDouble(baseWidth.getText()) : 0;
				a = Double.parseDouble(alpha.getText());
				segmentCount = Integer.parseInt(segments.getText());
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
			
			RobotSpecs specs = new RobotSpecs(maxVel, maxAccel, base);
			TrajectoryParams params = new TrajectoryParams();
			params.alpha = a;
			params.isTank = isTank.isSelected();
			params.pathType = selectedType;
			params.segmentCount = segmentCount;
			params.waypoints = waypointArray;
			
			JFrame pathFrame;
			JFrame movementFrame;
			double totalTime;
			
			if(isTank.isSelected()) {
				TankDriveTrajectory traj = new TankDriveTrajectory(new BasicTrajectory(specs, params));
				pathFrame = Grapher.graphPath(traj.getPath(), 1.0 / pathSamples);
				movementFrame = Grapher.graphTrajectory(traj, traj.totalTime() / trajSamples);
				totalTime = traj.totalTime();
			}
			else {
				BasicTrajectory traj = new BasicTrajectory(specs, params);
				pathFrame = Grapher.graphPath(traj.getPath(), 1.0 / pathSamples);
				movementFrame = Grapher.graphTrajectory(traj, traj.totalTime() / trajSamples);
				totalTime = traj.totalTime();
			}
			
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
			JOptionPane.showMessageDialog(mainFrame, "Trajectory Total Time: " + totalTime + " seconds");
			mainFrame.setVisible(false);
			movementFrame.setVisible(true);
			pathFrame.setVisible(true);
		});
		generateButton.setPreferredSize(buttonSize);
		buttonsPanel.add(generateButton);
		
		JPanel generatedCodePanel = new JPanel();
		JTextArea generatedCodeTextArea = new JTextArea();
		generatedCodeTextArea.setFont(new Font("monospaced", Font.PLAIN, 12));
		generatedCodeTextArea.setEditable(false);
		generatedCodeTextArea.setBackground(UIManager.getColor("Panel.background"));
		generatedCodeTextArea.setBorder(new JTextField().getBorder());
		generatedCodeTextArea.setTabSize(4);
		generatedCodePanel.add(generatedCodeTextArea);
		
		JButton generateCodeButton = new JButton("Get Code");
		generateCodeButton.addActionListener(e -> {
			
			double maxVel, maxAccel, base, a;
			int segmentCount;
			
			try {
				maxVel = Double.parseDouble(maxVelocity.getText());
				maxAccel = Double.parseDouble(maxAcceleration.getText());
				base = isTank.isSelected() ? Double.parseDouble(baseWidth.getText()) : 0;
				a = Double.parseDouble(alpha.getText());
				segmentCount = Integer.parseInt(segments.getText());
			}
			catch(NumberFormatException e1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: An invalid token was entered\nin one or more fields.", "Error", JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			if(waypoints.size() < 1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: You must specify at least one waypoint", "Error", JOptionPane.ERROR_MESSAGE);
				return;
			}
			if(waypoints.size() < 2) {
				int ret = JOptionPane.showConfirmDialog(mainFrame, "Warning: There are not enough waypoints to create a valid trajectory.\nAre you sure you want to continue?", "Warning", JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE);
				if(ret != JOptionPane.YES_OPTION)
					return;
			}
			
			StringBuilder generatedCode = new StringBuilder("RobotSpecs robotSpecs = new RobotSpecs(" + maxVel + ", " + maxAccel);
			if(isTank.isSelected()) {
				generatedCode.append(", " + base + ");\n");
			}
			else {
				generatedCode.append(");\n");
			}
			generatedCode.append("TrajectoryParams params = new TrajectoryParams();\n");
			generatedCode.append("params.waypoints = new Waypoint[] {\n");
			for(Waypoint w : waypoints) {
				double heading = w.getHeading();
				String angle = specialAngles.containsKey(heading) ? specialAngles.get(heading) : String.valueOf(heading);
				String waypointCode = "\t\tnew Waypoint(" + String.valueOf(w.getX()) + ", " + String.valueOf(w.getY()) + ", " + angle + "),\n";
				generatedCode.append(waypointCode);
			}
			generatedCode.append("};\n");
			generatedCode.append("params.alpha = " + a + ";\n");
			generatedCode.append("params.segmentCount = " + segmentCount + ";\n");
			generatedCode.append("params.isTank = " + isTank.isSelected() + ";\n");
			generatedCode.append("params.pathType = PathType." + selectedType.name() + ";\n");
			if(isTank.isSelected()) {
				generatedCode.append("TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);");
			}
			else {
				generatedCode.append("BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);");
			}
			
			String[] options = {
					"Copy to Clipboard",
					"Close",
			};
			generatedCodeTextArea.setText(generatedCode.toString());
			int ret = JOptionPane.showOptionDialog(mainFrame, generatedCodePanel, "Code", JOptionPane.YES_NO_OPTION, JOptionPane.PLAIN_MESSAGE, null, options, null);
			if(ret == JOptionPane.YES_OPTION) {
				Toolkit.getDefaultToolkit().getSystemClipboard().setContents(new StringSelection(generatedCode.toString()), null);
				JOptionPane.showMessageDialog(mainFrame, "Successfully copied to clipboard.", "Success", JOptionPane.INFORMATION_MESSAGE);
			}
		});
		generateCodeButton.setPreferredSize(buttonSize);
		buttonsPanel.add(generateCodeButton);
		
		menuBar = new JMenuBar();
		fileMenu = new JMenu("File");
		fileMenu.setMnemonic(KeyEvent.VK_F);
		menuBar.add(fileMenu);
		
		JMenuItem saveMenuItem = new JMenuItem("Save", KeyEvent.VK_S);
		saveMenuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, ActionEvent.CTRL_MASK));
		saveMenuItem.addActionListener(e -> {
			if(maxVelocity.getText().equals("") || maxAcceleration.getText().equals("") 
					|| baseWidth.getText().equals("") || alpha.getText().equals("")
					|| segments.getText().equals("")) {
				JOptionPane.showMessageDialog(mainFrame, "Error: Please fill in max velocity, max acceleration,\nbase width, alpha and segment count before saving.", "Error", JOptionPane.ERROR_MESSAGE);
				return;
			}
			
			double maxVel, maxAccel, base, a;
			int segmentCount;
			
			try {
				maxVel = Double.parseDouble(maxVelocity.getText());
				maxAccel = Double.parseDouble(maxAcceleration.getText());
				base = Double.parseDouble(baseWidth.getText());
				a = Double.parseDouble(alpha.getText());
				segmentCount = Integer.parseInt(segments.getText());
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
					out.write(maxVel + "," + maxAccel + "," + base + "," + a + "," + segmentCount + ",");
					switch(selectedType) {
					case QUINTIC_HERMITE:
						out.write(QHERMITE);
						break;
					case CUBIC_HERMITE:
						out.write(CHERMITE);
						break;
					case BEZIER:
						out.write(BEZIER);
						break;
					}
					out.write(",");
					if(isTank.isSelected()) {
						out.write("TankDriveTrajectory");
					}
					else {
						out.write("BasicTrajectory");
					}
					out.write("\n");
					
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
		});
		fileMenu.add(saveMenuItem);
		
		JMenuItem loadMenuItem = new JMenuItem("Load", KeyEvent.VK_L);
		loadMenuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_O, ActionEvent.CTRL_MASK));
		loadMenuItem.addActionListener(e -> {
			JFileChooser fc = new JFileChooser();
			fc.setDialogTitle("Load File...");
			fc.setAcceptAllFileFilterUsed(false);
			fc.setFileFilter(new CSVFilter());
			fc.setFileSelectionMode(JFileChooser.FILES_ONLY);
			
			int ret = fc.showOpenDialog(mainFrame);
			if(ret == JFileChooser.APPROVE_OPTION) {
				try(BufferedReader in = new BufferedReader(new FileReader(fc.getSelectedFile()))) {
					String[] parameters = in.readLine().split(",");
					if(parameters.length < 7) {
						JOptionPane.showMessageDialog(mainFrame, "Error: The file format is invalid.", "Error", JOptionPane.ERROR_MESSAGE);
						return;
					}
					maxVelocity.setText(parameters[0]);
					maxAcceleration.setText(parameters[1]);
					baseWidth.setText(parameters[2]);
					alpha.setText(parameters[3]);
					segments.setText(parameters[4]);
					
					switch(parameters[5].trim()) {
					case QHERMITE:
						selectedType = PathType.QUINTIC_HERMITE;
						quinticHermiteButton.setSelected(true);
						cubicHermiteButton.setSelected(false);
						bezierButton.setSelected(false);
						break;
					case CHERMITE:
						selectedType = PathType.CUBIC_HERMITE;
						quinticHermiteButton.setSelected(false);
						cubicHermiteButton.setSelected(true);
						bezierButton.setSelected(false);
						break;
					case BEZIER:
						selectedType = PathType.BEZIER;
						quinticHermiteButton.setSelected(false);
						cubicHermiteButton.setSelected(false);
						bezierButton.setSelected(true);
						break;
					default:
						JOptionPane.showMessageDialog(mainFrame, "Error: The file format is invalid.", "Error", JOptionPane.ERROR_MESSAGE);
						return;
					}
					if(parameters[6].trim().equals("TankDriveTrajectory")) {
						isTank.setSelected(true);
					}
					else {
						isTank.setSelected(false);
					}
					
					in.mark(512);
					String nextLine;
					if((nextLine = in.readLine()) == null || nextLine.equals(""))
						return;
					in.reset();
					
					waypoints.clear();
					WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
					tableModel.setRowCount(0);
					
					String line;
					while((line = in.readLine()) != null && !line.equals("")) {
						String[] point = line.split(",");
						Waypoint w = new Waypoint(Double.parseDouble(point[0]), Double.parseDouble(point[1]), Math.toRadians(constrainAngle(Double.parseDouble(point[2]))));
						waypoints.add(w);
						tableModel.addRow(point);
					}
				}
				catch (IOException e1) {
					e1.printStackTrace();
				}
				
			}
		});
		fileMenu.add(loadMenuItem);
		
		JMenu waypointMenu = new JMenu("Waypoint");
		
		JMenuItem addWaypointMenuItem = new JMenuItem("Add...");
		addWaypointMenuItem.addActionListener(addWaypointAction);
		waypointMenu.add(addWaypointMenuItem);
		
		JMenuItem editWaypointMenuItem = new JMenuItem("Edit...");
		editWaypointMenuItem.addActionListener(editWaypointAction);
		waypointMenu.add(editWaypointMenuItem);
		
		JMenuItem deleteWaypointMenuItem = new JMenuItem("Delete");
		deleteWaypointMenuItem.addActionListener(deleteWaypointAction);
		waypointMenu.add(deleteWaypointMenuItem);
		
		JMenuItem clearWaypointsMenuItem = new JMenuItem("Clear All");
		clearWaypointsMenuItem.addActionListener(e -> {
			int ret = JOptionPane.showConfirmDialog(mainFrame, "Are you sure you want to clear all waypoints?", "Are you sure?", JOptionPane.YES_NO_OPTION, JOptionPane.QUESTION_MESSAGE);
			if(ret == JOptionPane.NO_OPTION)
				return;
			
			waypoints.clear();
			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			tableModel.setRowCount(0);
		});
		waypointMenu.add(clearWaypointsMenuItem);
		menuBar.add(waypointMenu);
		
		JMenu settingsMenu = new JMenu("Settings");
		sampleCountPanel = new JPanel();
		sampleCountPanel.setLayout(new BoxLayout(sampleCountPanel, BoxLayout.Y_AXIS));
		sampleCountPanel.add(new JLabel("# of Samples for Paths"));
		sampleCountPanel.add(pathSamplesField);
		sampleCountPanel.add(new JLabel("# of Samples for Trajectories"));
		sampleCountPanel.add(trajSamplesField);
		JMenuItem sampleCountChange = new JMenuItem("Change Sample Count...");
		sampleCountChange.addActionListener(e -> {
			boolean pass = false;
			
			while(!pass) {
				pathSamplesField.setText(String.valueOf(pathSamples));
				trajSamplesField.setText(String.valueOf(trajSamples));
				int ret = JOptionPane.showConfirmDialog(mainFrame, sampleCountPanel, "Change Sample Count", JOptionPane.OK_CANCEL_OPTION);
				if(ret == JOptionPane.OK_OPTION) {
					try {
						int newPathSamples = Integer.parseInt(pathSamplesField.getText());
						int newTrajSamples = Integer.parseInt(trajSamplesField.getText());
						
						pathSamples = newPathSamples;
						trajSamples = newTrajSamples;
						pass = true;
					}
					catch(NumberFormatException nfe) {
						JOptionPane.showMessageDialog(mainFrame, "Error: An invalid token was entered\nin one or more fields.", "Error", JOptionPane.ERROR_MESSAGE);
						pass = false;
					}
				}
				else {
					pass = true;
				}
			}
		});
		settingsMenu.add(sampleCountChange);
		
		menuBar.add(settingsMenu);
		
		mainPanel.add(buttonsPanel, BorderLayout.PAGE_END);
		
		mainFrame = new JFrame("Path Parameters");
		mainFrame.setJMenuBar(menuBar);
		mainFrame.setContentPane(mainPanel);
		mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mainFrame.pack();
		mainFrame.setVisible(true);
	}

	
	public static void main(String[] args) {
		SwingUtilities.invokeLater(() -> {
			@SuppressWarnings("unused")
			TrajectoryVisualizationTool t = new TrajectoryVisualizationTool();
		});
	}
}
