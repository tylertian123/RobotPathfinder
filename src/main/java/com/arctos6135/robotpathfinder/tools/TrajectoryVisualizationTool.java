package com.arctos6135.robotpathfinder.tools;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Toolkit;
import java.awt.datatransfer.StringSelection;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.regex.Pattern;

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

import com.arctos6135.robotpathfinder.core.GlobalLibraryLoader;
import com.arctos6135.robotpathfinder.core.RobotSpecs;
import com.arctos6135.robotpathfinder.core.TrajectoryParams;
import com.arctos6135.robotpathfinder.core.Waypoint;
import com.arctos6135.robotpathfinder.core.path.Path;
import com.arctos6135.robotpathfinder.core.path.PathType;
import com.arctos6135.robotpathfinder.core.trajectory.BasicTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TankDriveTrajectory;
import com.arctos6135.robotpathfinder.core.trajectory.TrajectoryGenerationException;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonSyntaxException;

/**
 * A GUI tool built with Swing to help visualize trajectories.
 * 
 * @author Tyler Tian
 * @since 3.0.0
 */
public class TrajectoryVisualizationTool {
	/*
	 * This generates the most beautiful UI you've ever seen. It is also the biggest
	 * mess of code you've ever seen.
	 */

	// The main window frame
	static JFrame mainFrame;
	// The main window's panel
	static JPanel mainPanel;
	// The panel at the bottom of the main window's panel that holds all the buttons
	static JPanel buttonsPanel;

	// The text boxes in the add/edit waypoint dialog panel
	static JTextField waypointX = new JTextField();
	static JTextField waypointY = new JTextField();
	static JTextField waypointHeading = new JTextField();
	static JTextField waypointVelocity = new JTextField();
	// The add/edit waypoint dialog panel
	static JPanel waypointPanel;

	// The table that holds the waypoints
	static JTable table;

	// The number of samples for graphing paths and trajectories
	static int pathSamples = 200;
	static int trajSamples = 500;
	// Text boxes and panel for the change sample count dialog
	static JTextField pathSamplesField = new JTextField(String.valueOf(pathSamples));
	static JTextField trajSamplesField = new JTextField(String.valueOf(trajSamples));
	static JPanel sampleCountPanel;

	// Text boxes on the main screen
	static JTextField baseWidth = new JTextField("0");
	static JTextField alpha = new JTextField();
	static JTextField segments = new JTextField("1000");
	static JTextField maxVelocity = new JTextField();
	static JTextField maxAcceleration = new JTextField();
	// The JPanel that holds all the parameter textboxes above
	static JPanel argumentsPanel;

	static JMenuBar menuBar;
	static JMenu fileMenu;

	// Waypoints used for generation
	static ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();

	// Action commands for the radio buttons
	static final String QHERMITE = "quinticHermite";
	static final String CHERMITE = "cubicHermite";
	static final String BEZIER = "bezier";
	static JRadioButton quinticHermiteButton, cubicHermiteButton, bezierButton;
	static PathType selectedType = PathType.QUINTIC_HERMITE;

	static JCheckBox isTank;

	// Column names for the waypoint table
	static final String[] COLUMN_NAMES = new String[] { "X Position", "Y Position", "Robot Direction (Degrees)",
			"Velocity", };

	// This maps special angles in degrees to string representations of those angles
	// in radians
	// Used for codegen
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
		for (int i = 0; i < arr.length; i++)
			d[i] = arr[i];
		return d;
	}

	static double constrainAngle(double angle) {
		if (angle <= 180.0 && angle > -180.0)
			return angle;
		while (angle > 180.0) {
			angle -= 360.0;
		}
		while (angle <= -180.0) {
			angle += 360.0;
		}
		return angle;
	}

	// A file filter that only allows JSON files
	static class JsonFilter extends FileFilter {

		@Override
		public boolean accept(File f) {
			if (f.isDirectory())
				return true;
			try {
				String ext = f.getName().substring(f.getName().lastIndexOf("."));
				return ext.equals(".json");
			} catch (StringIndexOutOfBoundsException e) {
				return false;
			}
		}

		@Override
		public String getDescription() {
			return "JSON File (*.json)";
		}

	}

	// A file filter that only allows CSV files (for compatibility)
	static class CsvFilter extends FileFilter {

		@Override
		public boolean accept(File f) {
			if (f.isDirectory())
				return true;
			try {
				String ext = f.getName().substring(f.getName().lastIndexOf("."));
				return ext.equals(".csv");
			} catch (StringIndexOutOfBoundsException e) {
				return false;
			}
		}

		@Override
		public String getDescription() {
			return "Comma-Separated Values File (*.csv)";
		}

	}

	/**
	 * Custom table model for waypoints.
	 */
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
			// Make the cells all non-editable so only entire rows can be edited at a time
			// via the buttons
			return false;
		}
	}

	// Used for JSON generation
	static class TrajectoryVisualizerParameters {
		public double maxVelocity;
		public double maxAcceleration;
		public double basePlateWidth;
		public double alpha;
		public int sampleCount;

		public boolean tankDrive;
		public String pathType;

		public Waypoint[] waypoints;
	}

	static void saveAsJson(String path, double maxVel, double maxAccel, double baseWidth, double alpha, int sampleCount,
			PathType pathType) throws IOException {
		if (!path.endsWith(".json"))
			path += ".json";

		try (BufferedWriter out = new BufferedWriter(new FileWriter(path))) {
			TrajectoryVisualizerParameters params = new TrajectoryVisualizerParameters();
			params.maxAcceleration = maxAccel;
			params.maxVelocity = maxVel;
			params.basePlateWidth = baseWidth;
			params.alpha = alpha;
			params.sampleCount = sampleCount;

			switch (selectedType) {
			case QUINTIC_HERMITE:
				params.pathType = QHERMITE;
				break;
			case CUBIC_HERMITE:
				params.pathType = CHERMITE;
				break;
			case BEZIER:
				params.pathType = BEZIER;
				break;
			}
			Waypoint[] paramsWaypoints = new Waypoint[waypoints.size()];
			waypoints.toArray(paramsWaypoints);
			params.waypoints = paramsWaypoints;

			Gson gson = new GsonBuilder().serializeSpecialFloatingPointValues().setPrettyPrinting().create();
			out.write(gson.toJson(params).replace("NaN", "null"));
		} catch (IOException e) {
			throw e;
		}
	}

	static void loadJson(File file) throws IOException {
		try (BufferedReader in = new BufferedReader(new FileReader(file))) {
			// Read in the JSON
			Gson gson = new GsonBuilder().serializeSpecialFloatingPointValues().create();
			StringBuilder jsonBuilder = new StringBuilder();
			String line;
			while ((line = in.readLine()) != null) {
				jsonBuilder.append(line);
			}
			String json = jsonBuilder.toString();
			TrajectoryVisualizerParameters params;
			try {
				params = gson.fromJson(json, TrajectoryVisualizerParameters.class);
			} catch (JsonSyntaxException e1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: The file format is invalid.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}

			// Load the path type
			switch (params.pathType) {
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
				JOptionPane.showMessageDialog(mainFrame, "Error: The file format is invalid.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			maxVelocity.setText(String.valueOf(params.maxVelocity));
			maxAcceleration.setText(String.valueOf(params.maxAcceleration));
			baseWidth.setText(String.valueOf(params.basePlateWidth));
			alpha.setText(String.valueOf(params.alpha));
			segments.setText(String.valueOf(params.sampleCount));
			// Load the tank drive checkbox
			if (params.tankDrive) {
				isTank.setSelected(true);
			} else {
				isTank.setSelected(false);
			}
			// Load the waypoints
			waypoints.clear();
			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			tableModel.setRowCount(0);

			// Fill in the table
			waypoints = new ArrayList<Waypoint>(Arrays.asList(params.waypoints));
			for (Waypoint wp : waypoints) {
				if (!Double.isNaN(wp.getVelocity())) {
					tableModel.addRow(new String[] { String.valueOf(wp.getX()), String.valueOf(wp.getY()),
							String.valueOf(Math.toDegrees(wp.getHeading())), String.valueOf(wp.getVelocity()) });
				} else {
					tableModel.addRow(new String[] { String.valueOf(wp.getX()), String.valueOf(wp.getY()),
							String.valueOf(Math.toDegrees(wp.getHeading())), "unconstrained" });
				}
			}
		} catch (IOException e) {
			throw e;
		}
	}

	static void saveAsCsv(String path, double maxVel, double maxAccel, double baseWidth, double alpha, int sampleCount,
			PathType pathType) throws IOException {
		if (!path.endsWith(".csv"))
			path += ".csv";

		try (BufferedWriter out = new BufferedWriter(new FileWriter(path))) {
			out.write(maxVel + "," + maxAccel + "," + baseWidth + "," + alpha + "," + sampleCount + ",");
			switch (selectedType) {
			case QUINTIC_HERMITE:
				out.write("Q");
				break;
			case CUBIC_HERMITE:
				out.write("C");
				break;
			case BEZIER:
				out.write("B");
				break;
			}
			out.write(",");
			if (isTank.isSelected()) {
				out.write("TankDriveTrajectory");
			} else {
				out.write("BasicTrajectory");
			}
			out.write("\n");

			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			for (int row = 0; row < tableModel.getRowCount(); row++) {
				String x = (String) (tableModel.getValueAt(row, 0));
				String y = (String) (tableModel.getValueAt(row, 1));
				String heading = (String) (tableModel.getValueAt(row, 2));
				String velocity = (String) (tableModel.getValueAt(row, 3));

				if (velocity.equals("unconstrained")) {
					out.write(x + "," + y + "," + heading + "\n");
				} else {
					out.write(x + "," + y + "," + heading + "," + velocity + "\n");
				}
			}
		} catch (IOException e) {
			throw e;
		}
	}

	static void loadCsv(File file) throws IOException {
		try (BufferedReader in = new BufferedReader(new FileReader(file))) {
			String[] parameters = in.readLine().split(",");
			if (parameters.length < 7) {
				JOptionPane.showMessageDialog(mainFrame, "Error: The file format is invalid.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			maxVelocity.setText(parameters[0]);
			maxAcceleration.setText(parameters[1]);
			baseWidth.setText(parameters[2]);
			alpha.setText(parameters[3]);
			segments.setText(parameters[4]);

			switch (parameters[5].trim()) {
			case "Q":
				selectedType = PathType.QUINTIC_HERMITE;
				quinticHermiteButton.setSelected(true);
				cubicHermiteButton.setSelected(false);
				bezierButton.setSelected(false);
				break;
			case "C":
				selectedType = PathType.CUBIC_HERMITE;
				quinticHermiteButton.setSelected(false);
				cubicHermiteButton.setSelected(true);
				bezierButton.setSelected(false);
				break;
			case "B":
				selectedType = PathType.BEZIER;
				quinticHermiteButton.setSelected(false);
				cubicHermiteButton.setSelected(false);
				bezierButton.setSelected(true);
				break;
			default:
				JOptionPane.showMessageDialog(mainFrame, "Error: The file format is invalid.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			if (parameters[6].trim().equals("TankDriveTrajectory")) {
				isTank.setSelected(true);
			} else {
				isTank.setSelected(false);
			}

			waypoints.clear();
			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			tableModel.setRowCount(0);

			String line;
			while ((line = in.readLine()) != null && !line.equals("")) {
				String[] point = line.split(",");
				Waypoint w = new Waypoint(Double.parseDouble(point[0]), Double.parseDouble(point[1]),
						Math.toRadians(constrainAngle(Double.parseDouble(point[2]))),
						point.length < 4 ? Double.NaN : Double.parseDouble(point[3]));
				waypoints.add(w);

				tableModel.addRow(
						point.length == 4 ? point : new String[] { point[0], point[1], point[2], "unconstrained" });
			}
		} catch (IOException e) {
			throw e;
		}
	}

	TrajectoryVisualizationTool() {
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch (ClassNotFoundException | InstantiationException | IllegalAccessException
				| UnsupportedLookAndFeelException e) {
			e.printStackTrace();
		}

		// Set up the waypoints dialog
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
		waypointPanel.add(new JLabel("Velocity (Optional)"));
		waypointPanel.add(waypointVelocity);

		// Set up the main JPanel
		mainPanel = new JPanel();
		mainPanel.setLayout(new BorderLayout());
		// Waypoint table
		table = new JTable(new WaypointTableModel(COLUMN_NAMES, 0));
		table.setCellSelectionEnabled(false);
		table.setRowSelectionAllowed(true);
		table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		// Wrap a JScrollPane around the table to make it scrollable
		JScrollPane scrollPane = new JScrollPane(table);
		scrollPane.setPreferredSize(new Dimension(620, 300));
		table.setFillsViewportHeight(true);
		mainPanel.add(scrollPane, BorderLayout.PAGE_START);

		// Set up parameters section
		argumentsPanel = new JPanel();
		Dimension textFieldSize = new Dimension(75, 20);
		maxVelocity.setPreferredSize(textFieldSize);
		maxAcceleration.setPreferredSize(textFieldSize);
		baseWidth.setPreferredSize(textFieldSize);
		alpha.setPreferredSize(textFieldSize);
		segments.setPreferredSize(textFieldSize);

		// Use a bunch of sub-panels to make the parameter boxes line up
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
		// Attach a special action listener to the change path type radio buttons
		ActionListener changePathType = e -> {
			switch (e.getActionCommand()) {
			case QHERMITE:
				selectedType = PathType.QUINTIC_HERMITE;
				break;
			case CHERMITE:
				selectedType = PathType.CUBIC_HERMITE;
				break;
			case BEZIER:
				selectedType = PathType.BEZIER;
				break;
			default:
				System.err.println("Error: Unrecognized path type");
				break;
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

		// Set up buttons
		Dimension buttonSize = new Dimension(120, 30);
		JButton addWaypointButton = new JButton("New Waypoint");
		// Add waypoint button action listener
		ActionListener addWaypointAction = e -> {
			boolean error = false;
			do {
				int selectedRow = table.getSelectedRow();
				// Show the custom dialog
				int response = JOptionPane.showConfirmDialog(mainFrame, waypointPanel, "New Waypoint...",
						JOptionPane.OK_CANCEL_OPTION);
				if (response != JOptionPane.OK_OPTION)
					break;
				try {
					// Try and parse all the arguments
					double x = Double.parseDouble(waypointX.getText());
					double y = Double.parseDouble(waypointY.getText());
					double heading = Double.parseDouble(waypointHeading.getText());
					// If velocity is not specified, set it to NaN
					double velocity = waypointVelocity.getText().length() > 0
							? Double.parseDouble(waypointVelocity.getText())
							: Double.NaN;

					WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
					// selectedRow = -1 when there is no row selected
					if (selectedRow == -1) {
						waypoints.add(new Waypoint(x, y, Math.toRadians(constrainAngle(heading)), velocity));
						tableModel.addRow(
								new Object[] { waypointX.getText(), waypointY.getText(), waypointHeading.getText(),
										Double.isNaN(velocity) ? "unconstrained" : waypointVelocity.getText() });
					}
					// If a row is selected then insert the row and the waypoint at the correct
					// location
					else {
						waypoints.add(selectedRow,
								new Waypoint(x, y, Math.toRadians(constrainAngle(heading)), velocity));
						tableModel.insertRow(selectedRow,
								new Object[] { waypointX.getText(), waypointY.getText(), waypointHeading.getText(),
										Double.isNaN(velocity) ? "unconstrained" : waypointVelocity.getText() });
					}

					error = false;
				} catch (NumberFormatException e1) {
					error = true;
					JOptionPane.showMessageDialog(mainFrame,
							"Error: An invalid token was entered\nin one or more fields.", "Error",
							JOptionPane.ERROR_MESSAGE);
				}
			} while (error);

			// Clear the contents of the text boxes afterwards
			waypointX.setText("");
			waypointY.setText("");
			waypointHeading.setText("");
			waypointVelocity.setText("");
		};
		addWaypointButton.addActionListener(addWaypointAction);
		addWaypointButton.setPreferredSize(buttonSize);
		buttonsPanel.add(addWaypointButton);

		// Edit waypoint button
		JButton editWaypointButton = new JButton("Edit Waypoint");
		ActionListener editWaypointAction = e -> {
			int index = table.getSelectedRow();
			if (index == -1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: No waypoint selected.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}

			// Grab the values from the row and put them into the dialog's text boxes
			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			String xStr = (String) tableModel.getValueAt(index, 0);
			String yStr = (String) tableModel.getValueAt(index, 1);
			String headingStr = (String) tableModel.getValueAt(index, 2);
			String velocityStr = (String) tableModel.getValueAt(index, 3);

			waypointX.setText(xStr);
			waypointY.setText(yStr);
			waypointHeading.setText(headingStr);
			waypointVelocity.setText(velocityStr.equals("unconstrained") ? "" : velocityStr);

			boolean error;
			do {
				int response = JOptionPane.showConfirmDialog(mainFrame, waypointPanel, "Edit Waypoint...",
						JOptionPane.OK_CANCEL_OPTION);
				if (response != JOptionPane.OK_OPTION)
					break;

				try {
					// Once again parse all the args
					double x = Double.parseDouble(waypointX.getText());
					double y = Double.parseDouble(waypointY.getText());
					double heading = Double.parseDouble(waypointHeading.getText());
					double velocity = waypointVelocity.getText().length() > 0
							? Double.parseDouble(waypointVelocity.getText())
							: Double.NaN;

					// Update the table and waypoint
					waypoints.set(index, new Waypoint(x, y, Math.toRadians(constrainAngle(heading)), velocity));
					tableModel.setValueAt(waypointX.getText(), index, 0);
					tableModel.setValueAt(waypointY.getText(), index, 1);
					tableModel.setValueAt(waypointHeading.getText(), index, 2);
					tableModel.setValueAt(Double.isNaN(velocity) ? "unconstrained" : waypointVelocity.getText(), index,
							3);

					error = false;
				} catch (NumberFormatException e1) {
					error = true;
					JOptionPane.showMessageDialog(mainFrame,
							"Error: An invalid token was entered\nin one or more fields.", "Error",
							JOptionPane.ERROR_MESSAGE);
				}
			} while (error);

			waypointX.setText("");
			waypointY.setText("");
			waypointHeading.setText("");
			waypointVelocity.setText("");
		};
		editWaypointButton.addActionListener(editWaypointAction);
		editWaypointButton.setPreferredSize(buttonSize);
		buttonsPanel.add(editWaypointButton);

		// Delete waypoint button
		JButton deleteWaypointButton = new JButton("Remove Waypoint");
		ActionListener deleteWaypointAction = e -> {
			int index = table.getSelectedRow();
			if (index == -1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: No waypoint selected.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}

			waypoints.remove(index);
			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			tableModel.removeRow(index);
		};
		deleteWaypointButton.addActionListener(deleteWaypointAction);
		deleteWaypointButton.setPreferredSize(buttonSize);
		buttonsPanel.add(deleteWaypointButton);

		// Preview button
		// The preview only generates the path and not the trajectory
		JButton previewButton = new JButton("Preview");
		previewButton.addActionListener(e -> {
			double base, a;

			try {
				// Previewing only needs a base width and alpha
				base = isTank.isSelected() ? Double.parseDouble(baseWidth.getText()) : 0;
				a = Double.parseDouble(alpha.getText());
			} catch (NumberFormatException e1) {
				JOptionPane
						.showMessageDialog(mainFrame,
								isTank.isSelected() ? "Error: Please enter a valid alpha value and base plate width."
										: "Error: Please enter a valid alpha value.",
								"Error", JOptionPane.ERROR_MESSAGE);
				return;
			}

			if (waypoints.size() < 2) {
				JOptionPane.showMessageDialog(mainFrame, "Error: Not enough waypoints.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}

			Waypoint[] waypointArray = new Waypoint[waypoints.size()];
			waypointArray = waypoints.toArray(waypointArray);

			Path path = new Path(waypointArray, a, selectedType);
			path.setBaseRadius(base / 2);

			JFrame pathFrame = Grapher.graphPath(path, 1.0 / pathSamples);
			// Before showing the graphed window, add a listener to it
			pathFrame.addWindowListener(new WindowAdapter() {
				@Override
				public void windowClosing(WindowEvent e) {
					// Show the main window again
					mainFrame.setVisible(true);
				}
			});
			// Hide the main window and show the grapher window
			mainFrame.setVisible(false);
			pathFrame.setVisible(true);
		});
		previewButton.setPreferredSize(buttonSize);
		buttonsPanel.add(previewButton);

		// Generate button
		JButton generateButton = new JButton("Generate");
		generateButton.addActionListener(e -> {
			double maxVel, maxAccel, base, a;
			int sampleCount;

			try {
				// Parse all args
				maxVel = Double.parseDouble(maxVelocity.getText());
				maxAccel = Double.parseDouble(maxAcceleration.getText());
				base = isTank.isSelected() ? Double.parseDouble(baseWidth.getText()) : 0;
				a = Double.parseDouble(alpha.getText());
				sampleCount = Integer.parseInt(segments.getText());
			} catch (NumberFormatException e1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: An invalid token was entered\nin one or more fields.",
						"Error", JOptionPane.ERROR_MESSAGE);
				return;
			}

			if (waypoints.size() < 2) {
				JOptionPane.showMessageDialog(mainFrame, "Error: Not enough waypoints.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			if (sampleCount < 1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: Sample count cannot be zero.", "Error",
						JOptionPane.ERROR_MESSAGE);
			}

			Waypoint[] waypointArray = new Waypoint[waypoints.size()];
			waypointArray = waypoints.toArray(waypointArray);

			// Generate the path and trajectory
			RobotSpecs specs = new RobotSpecs(maxVel, maxAccel, base);
			TrajectoryParams params = new TrajectoryParams();
			params.alpha = a;
			params.pathType = selectedType;
			params.sampleCount = sampleCount;
			params.waypoints = waypointArray;

			JFrame pathFrame;
			JFrame movementFrame;
			double totalTime;

			// Generate the correct trajectory based on options and graph it
			try {
				if (isTank.isSelected()) {
					TankDriveTrajectory traj = new TankDriveTrajectory(specs, params);
					pathFrame = Grapher.graphPath(traj.getPath(), 1.0 / pathSamples);
					movementFrame = Grapher.graphTrajectory(traj, traj.totalTime() / trajSamples);
					totalTime = traj.totalTime();
				} else {
					BasicTrajectory traj = new BasicTrajectory(specs, params);
					pathFrame = Grapher.graphPath(traj.getPath(), 1.0 / pathSamples);
					movementFrame = Grapher.graphTrajectory(traj, traj.totalTime() / trajSamples);
					totalTime = traj.totalTime();
				}
			} catch (TrajectoryGenerationException e1) {
				JOptionPane
						.showMessageDialog(mainFrame,
								"An error occurred during trajectory generation:\n" + e1.getMessage()
										+ "\nPlease adjust the constraints and try again.",
								"Error", JOptionPane.ERROR_MESSAGE);
				return;
			} catch (ArrayIndexOutOfBoundsException e1) {
				JOptionPane.showMessageDialog(mainFrame, "A zero length trajectory cannot be graphed.", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}

			// Add the window close hook that shows the main window back
			WindowAdapter closeHook = new WindowAdapter() {
				@Override
				public void windowClosing(WindowEvent e) {
					pathFrame.setVisible(false);
					movementFrame.setVisible(false);
					pathFrame.dispose();
					movementFrame.dispose();
					mainFrame.setVisible(true);
				}
			};
			pathFrame.addWindowListener(closeHook);
			movementFrame.addWindowListener(closeHook);

			// Hide the main window and show the time dialog
			JOptionPane.showMessageDialog(mainFrame, "Trajectory Total Time: " + totalTime + " seconds");
			mainFrame.setVisible(false);
			movementFrame.setVisible(true);
			pathFrame.setVisible(true);
		});
		generateButton.setPreferredSize(buttonSize);
		buttonsPanel.add(generateButton);

		// Codegen dialog
		JPanel generatedCodePanel = new JPanel();
		JTextArea generatedCodeTextArea = new JTextArea();
		// Set the font to be monospaced and configure other properties
		generatedCodeTextArea.setFont(new Font("monospaced", Font.PLAIN, 12));
		generatedCodeTextArea.setEditable(false);
		generatedCodeTextArea.setBackground(UIManager.getColor("Panel.background"));
		generatedCodeTextArea.setBorder(new JTextField().getBorder());
		generatedCodeTextArea.setTabSize(4);
		generatedCodePanel.add(generatedCodeTextArea);

		JButton generateCodeButton = new JButton("Get Code");
		generateCodeButton.addActionListener(e -> {

			double maxVel, maxAccel, base, a;
			int sampleCount;

			try {
				// Parse all params
				maxVel = Double.parseDouble(maxVelocity.getText());
				maxAccel = Double.parseDouble(maxAcceleration.getText());
				base = isTank.isSelected() ? Double.parseDouble(baseWidth.getText()) : 0;
				a = Double.parseDouble(alpha.getText());
				sampleCount = Integer.parseInt(segments.getText());
			} catch (NumberFormatException e1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: An invalid token was entered\nin one or more fields.",
						"Error", JOptionPane.ERROR_MESSAGE);
				return;
			}

			if (waypoints.size() < 1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: You must specify at least one waypoint", "Error",
						JOptionPane.ERROR_MESSAGE);
				return;
			}
			if (waypoints.size() < 2) {
				int ret = JOptionPane.showConfirmDialog(mainFrame,
						"Warning: There are not enough waypoints to create a valid trajectory.\nAre you sure you want to continue?",
						"Warning", JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE);
				if (ret != JOptionPane.YES_OPTION)
					return;
			}

			// Start codegen
			StringBuilder generatedCode = new StringBuilder(
					"RobotSpecs robotSpecs = new RobotSpecs(" + maxVel + ", " + maxAccel);
			if (isTank.isSelected()) {
				generatedCode.append(", " + base + ");\n");
			} else {
				generatedCode.append(");\n");
			}
			generatedCode.append("TrajectoryParams params = new TrajectoryParams();\n");
			// Generate each waypoint
			generatedCode.append("params.waypoints = new Waypoint[] {\n");
			for (Waypoint w : waypoints) {
				double heading = w.getHeading();
				String angle = specialAngles.containsKey(heading) ? specialAngles.get(heading)
						: String.valueOf(heading);
				// Create different code for WaypointEx
				String waypointCode = Double.isNaN(w.getVelocity())
						? "\tnew Waypoint(" + w.getX() + ", " + w.getY() + ", " + angle + ", " + w.getVelocity()
								+ "),\n"
						: "\tnew Waypoint(" + w.getX() + ", " + w.getY() + ", " + angle + "),\n";
				generatedCode.append(waypointCode);
			}
			generatedCode.append("};\n");
			generatedCode.append("params.alpha = " + a + ";\n");
			generatedCode.append("params.sampleCount = " + sampleCount + ";\n");
			generatedCode.append("params.pathType = PathType." + selectedType.name() + ";\n");
			generatedCode.append("// Don't forget to call close() or free()!\n");
			if (isTank.isSelected()) {
				generatedCode.append("TankDriveTrajectory trajectory = new TankDriveTrajectory(robotSpecs, params);");
			} else {
				generatedCode.append("BasicTrajectory trajectory = new BasicTrajectory(robotSpecs, params);");
			}

			String[] options = { "Copy to Clipboard", "Close", };
			generatedCodeTextArea.setText(generatedCode.toString());
			int ret = JOptionPane.showOptionDialog(mainFrame, generatedCodePanel, "Code", JOptionPane.YES_NO_OPTION,
					JOptionPane.PLAIN_MESSAGE, null, options, null);
			// If copy was selected, copy to the clipboard and display a message box
			if (ret == JOptionPane.YES_OPTION) {
				Toolkit.getDefaultToolkit().getSystemClipboard()
						.setContents(new StringSelection(generatedCode.toString()), null);
				JOptionPane.showMessageDialog(mainFrame, "Successfully copied to clipboard.", "Success",
						JOptionPane.INFORMATION_MESSAGE);
			}
		});
		generateCodeButton.setPreferredSize(buttonSize);
		buttonsPanel.add(generateCodeButton);

		menuBar = new JMenuBar();
		fileMenu = new JMenu("File");
		fileMenu.setMnemonic(KeyEvent.VK_F);
		menuBar.add(fileMenu);
		// Save menu
		JMenuItem saveMenuItem = new JMenuItem("Save", KeyEvent.VK_S);
		saveMenuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, ActionEvent.CTRL_MASK));
		saveMenuItem.addActionListener(e -> {
			if (maxVelocity.getText().equals("") || maxAcceleration.getText().equals("")
					|| baseWidth.getText().equals("") || alpha.getText().equals("") || segments.getText().equals("")) {
				JOptionPane.showMessageDialog(mainFrame,
						"Error: Please fill in max velocity, max acceleration,\nbase width, alpha and segment count before saving.",
						"Error", JOptionPane.ERROR_MESSAGE);
				return;
			}

			double maxVel, maxAccel, base, a;
			int sampleCount;

			try {
				maxVel = Double.parseDouble(maxVelocity.getText());
				maxAccel = Double.parseDouble(maxAcceleration.getText());
				base = Double.parseDouble(baseWidth.getText());
				a = Double.parseDouble(alpha.getText());
				sampleCount = Integer.parseInt(segments.getText());
			} catch (NumberFormatException e1) {
				JOptionPane.showMessageDialog(mainFrame, "Error: An invalid token was entered\nin one or more fields.",
						"Error", JOptionPane.ERROR_MESSAGE);
				return;
			}

			// Choose a json to save
			JFileChooser fc = new JFileChooser();
			fc.setDialogTitle("Save As...");
			fc.setAcceptAllFileFilterUsed(false);
			fc.addChoosableFileFilter(new JsonFilter());
			fc.addChoosableFileFilter(new CsvFilter());
			fc.setFileSelectionMode(JFileChooser.FILES_ONLY);

			int ret = fc.showSaveDialog(mainFrame);
			if (ret == JFileChooser.APPROVE_OPTION) {
				String path = fc.getSelectedFile().getAbsolutePath();
				try {
					// Maintain legacy support for CSV
					if (fc.getFileFilter() instanceof JsonFilter) {
						saveAsJson(path, maxVel, maxAccel, base, a, sampleCount, selectedType);
					} else {
						saveAsCsv(path, maxVel, maxAccel, base, a, sampleCount, selectedType);
					}
					JOptionPane.showMessageDialog(mainFrame, "Data saved successfully.", "Success",
							JOptionPane.INFORMATION_MESSAGE);
				} catch (IOException e1) {
					e1.printStackTrace();
					JOptionPane.showMessageDialog(mainFrame, "Failed to save data!", "Error",
							JOptionPane.ERROR_MESSAGE);
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
			fc.addChoosableFileFilter(new JsonFilter());
			fc.addChoosableFileFilter(new CsvFilter());
			fc.setFileSelectionMode(JFileChooser.FILES_ONLY);

			int ret = fc.showOpenDialog(mainFrame);
			if (ret == JFileChooser.APPROVE_OPTION) {
				try {
					if (fc.getFileFilter() instanceof JsonFilter) {
						loadJson(fc.getSelectedFile());
					} else {
						loadCsv(fc.getSelectedFile());
					}
				} catch (IOException e1) {
					JOptionPane.showMessageDialog(mainFrame, "Load Failed!", "Error", JOptionPane.ERROR_MESSAGE);
				}
			}
		});
		fileMenu.add(loadMenuItem);

		JMenu waypointMenu = new JMenu("Waypoint");

		JMenuItem addWaypointMenuItem = new JMenuItem("New...");
		addWaypointMenuItem.addActionListener(addWaypointAction);
		addWaypointMenuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_N, ActionEvent.CTRL_MASK));
		waypointMenu.add(addWaypointMenuItem);

		JMenuItem editWaypointMenuItem = new JMenuItem("Edit...");
		editWaypointMenuItem.addActionListener(editWaypointAction);
		editWaypointMenuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_E, ActionEvent.CTRL_MASK));
		waypointMenu.add(editWaypointMenuItem);

		JMenuItem deleteWaypointMenuItem = new JMenuItem("Delete");
		deleteWaypointMenuItem.addActionListener(deleteWaypointAction);
		deleteWaypointMenuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE, 0));
		waypointMenu.add(deleteWaypointMenuItem);

		JMenuItem clearWaypointsMenuItem = new JMenuItem("Clear All");
		clearWaypointsMenuItem.addActionListener(e -> {
			int ret = JOptionPane.showConfirmDialog(mainFrame, "Are you sure you want to clear all waypoints?",
					"Are you sure?", JOptionPane.YES_NO_OPTION, JOptionPane.QUESTION_MESSAGE);
			if (ret == JOptionPane.NO_OPTION)
				return;

			waypoints.clear();
			WaypointTableModel tableModel = (WaypointTableModel) table.getModel();
			tableModel.setRowCount(0);
		});
		clearWaypointsMenuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_DELETE, ActionEvent.CTRL_MASK));
		waypointMenu.add(clearWaypointsMenuItem);

		JMenuItem deselectWaypointMenuItem = new JMenuItem("Deselect");
		deselectWaypointMenuItem.addActionListener(e -> {
			table.clearSelection();
		});
		deselectWaypointMenuItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE, 0));
		waypointMenu.add(deselectWaypointMenuItem);

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

			while (!pass) {
				pathSamplesField.setText(String.valueOf(pathSamples));
				trajSamplesField.setText(String.valueOf(trajSamples));
				int ret = JOptionPane.showConfirmDialog(mainFrame, sampleCountPanel, "Change Sample Count",
						JOptionPane.OK_CANCEL_OPTION);
				if (ret == JOptionPane.OK_OPTION) {
					try {
						int newPathSamples = Integer.parseInt(pathSamplesField.getText());
						int newTrajSamples = Integer.parseInt(trajSamplesField.getText());

						pathSamples = newPathSamples;
						trajSamples = newTrajSamples;
						pass = true;
					} catch (NumberFormatException nfe) {
						JOptionPane.showMessageDialog(mainFrame,
								"Error: An invalid token was entered\nin one or more fields.", "Error",
								JOptionPane.ERROR_MESSAGE);
						pass = false;
					}
				} else {
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
		try {
			GlobalLibraryLoader.load();
		} catch (UnsatisfiedLinkError ule) {
		}

		if (!GlobalLibraryLoader.libraryLoaded()) {
			StringBuilder str = new StringBuilder("Failed to load dynamic library '"
					+ System.mapLibraryName("RobotPathfinder") + "'!'\nPlease ensure that the library is in the same"
					+ " directory as where this program was launched\n(" + System.getProperty("user.dir")
					+ File.separator + System.mapLibraryName("RobotPathfinder") + ") or present in one of the"
					+ " following locations:\n");

			for (String path : System.getProperty("java.library.path").split(Pattern.quote(File.pathSeparator))) {
				str.append(path);
				str.append('\n');
			}
			SwingUtilities.invokeLater(() -> {
				try {
					UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
				} catch (ClassNotFoundException | InstantiationException | IllegalAccessException
						| UnsupportedLookAndFeelException e) {
					e.printStackTrace();
				}
				JOptionPane.showMessageDialog(null, str.toString(), "Error", JOptionPane.ERROR_MESSAGE);
			});
		} else {
			SwingUtilities.invokeLater(() -> {
				@SuppressWarnings("unused")
				TrajectoryVisualizationTool t = new TrajectoryVisualizationTool();
			});
		}
	}
}
