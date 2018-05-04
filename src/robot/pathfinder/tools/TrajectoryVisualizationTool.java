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
	static JPanel optionsPanel;
	
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
		optionsPanel = new JPanel();
		optionsPanel.setLayout(new BoxLayout(optionsPanel, BoxLayout.PAGE_AXIS));
		optionsPanel.add(new JLabel("Waypoint X"));
		optionsPanel.add(waypointX);
		optionsPanel.add(new JLabel("Waypoint Y"));
		optionsPanel.add(waypointY);
		optionsPanel.add(new JLabel("Robot Direction (Degrees)"));
		optionsPanel.add(waypointHeading);
		
		mainPanel = new JPanel();
		mainPanel.setLayout(new BorderLayout());
		JTable table = new JTable(new WaypointTableModel(COLUMN_NAMES, 0));
		table.setCellSelectionEnabled(false);
		table.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		JScrollPane scrollPane = new JScrollPane(table);
		scrollPane.setPreferredSize(new Dimension(480, 300));
		table.setFillsViewportHeight(true);
		mainPanel.add(scrollPane, BorderLayout.PAGE_START);
		
		buttonsPanel = new JPanel();
		
		JButton addWaypointButton = new JButton("Add Waypoint");
		addWaypointButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				boolean error = false;
				do {
					int selectedRow = table.getSelectedRow();
					
					int response = JOptionPane.showConfirmDialog(mainFrame, optionsPanel, "New Waypoint...", JOptionPane.OK_CANCEL_OPTION);
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
