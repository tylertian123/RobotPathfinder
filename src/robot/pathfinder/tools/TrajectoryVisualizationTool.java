package robot.pathfinder.tools;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.BoxLayout;
import javax.swing.JButton;
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
import javax.swing.table.DefaultTableModel;

import robot.pathfinder.Waypoint;

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
	static JPanel argumentsPanel;
	
	static ArrayList<Waypoint> waypoints = new ArrayList<Waypoint>();
	
	static final String[] COLUMN_NAMES = new String[] {
			"X Position",
			"Y Position",
			"Robot Direction (Degrees)"
	};
	
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
		scrollPane.setPreferredSize(new Dimension(480, 300));
		table.setFillsViewportHeight(true);
		mainPanel.add(scrollPane, BorderLayout.PAGE_START);
		
		argumentsPanel = new JPanel();
		maxVelocity = new JTextField();
		maxAcceleration = new JTextField();
		baseWidth = new JTextField();
		alpha = new JTextField();
		segments = new JTextField();
		Dimension textFieldSize = new Dimension(75, 20);
		maxVelocity.setPreferredSize(textFieldSize);
		//maxVelocity.setMaximumSize(textFieldSize);
		maxAcceleration.setPreferredSize(textFieldSize);
		//maxAcceleration.setMaximumSize(textFieldSize);
		baseWidth.setPreferredSize(textFieldSize);
		//baseWidth.setMaximumSize(textFieldSize);
		alpha.setPreferredSize(textFieldSize);
		//alpha.setMaximumSize(textFieldSize);
		segments.setPreferredSize(textFieldSize);
		//segments.setMaximumSize(textFieldSize);
		
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
		
		argumentsPanel.add(subPanel1);
		argumentsPanel.add(subPanel2);
		argumentsPanel.add(subPanel3);
		
		mainPanel.add(argumentsPanel, BorderLayout.CENTER);
		
		buttonsPanel = new JPanel();
		
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
		addWaypointButton.setPreferredSize(new Dimension(120, 30));
		buttonsPanel.add(addWaypointButton);
		
		JButton deleteWaypointButton = new JButton("Delete Waypoint");
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
		deleteWaypointButton.setPreferredSize(new Dimension(120, 30));
		buttonsPanel.add(deleteWaypointButton);
		
		JButton generateButton = new JButton("Generate");
		generateButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				double maxVel, maxAccel, base, a, segmentCount;
				
				try {
					maxVel = Double.parseDouble(maxVelocity.getText());
					maxAccel = Double.parseDouble(maxAcceleration.getText());
					base = Double.parseDouble(baseWidth.getText());
					a = Double.parseDouble(alpha.getText());
					segmentCount = Integer.parseInt(segments.getText());
				}
				catch(NumberFormatException e1) {
					JOptionPane.showMessageDialog(mainFrame, "Error: No waypoint selected.", "Error", JOptionPane.ERROR_MESSAGE);
				}
			}
		});
		
		JButton exitButton = new JButton("Exit");
		exitButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				mainFrame.dispose();
				System.exit(0);
			}
		});
		exitButton.setPreferredSize(new Dimension(120, 30));
		buttonsPanel.add(exitButton);
		
		mainPanel.add(buttonsPanel, BorderLayout.PAGE_END);
		
		mainFrame = new JFrame("Path Parameters");
		mainFrame.setContentPane(mainPanel);
		mainFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		mainFrame.pack();
		mainFrame.setVisible(true);
	}

}
