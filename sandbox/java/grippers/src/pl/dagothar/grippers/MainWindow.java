/**
 * @author Adam Wolniakowski
 */

package pl.dagothar.grippers;



import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.logging.*;



/**
 * Main application window class
 */
public class MainWindow extends JFrame {
	// instance
	private static MainWindow _instance = null;
	
	// GUI elements
	JTextArea _logArea;
	
	/**
	 * Get MainWindow instance
	 */
	public static MainWindow getInstance() {
		if (_instance == null) {
			_instance = new MainWindow("Gripper evaluation");
		}
		
		return _instance;
	}
	
	/**
	 * Default constructor
	 */
	protected MainWindow(String name) {
		super(name);

		initialize();
	}
	
	/**
	 * Initializes GUI
	 */
	private void initialize() {
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setSize(640, 480);
		
		// setup layout
		Container contentPane = getContentPane();
		BorderLayout layout = new BorderLayout();
		contentPane.setLayout(layout);
		
		// create menu
		MainMenu menuBar = new MainMenu();
		setJMenuBar(menuBar);
		
		// setup log area
		this._logArea = new JTextArea(5, 80);
		this._logArea.setEditable(false);
		this._logArea.setLineWrap(true);
		JScrollPane logScrollPane = new JScrollPane(this._logArea);
		logScrollPane.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		getContentPane().add(logScrollPane, BorderLayout.PAGE_END);
		
		setVisible(true);
	}
	
	/**
	 * Create new gripper
	 */
	public void newGripper() {
		GripperBuilder gripperBuilder = new GripperBuilder();
		Gripper newGripper = gripperBuilder.name("lol").buildGripper();
		message("New gripper created.\n");
		message(newGripper.toString() + "\n");
	}
	
	/**
	 * Shows about dialog
	 */
	public void about() {
	}
	
	/**
	 * Simple log message
	 */
	public void message(String msg) {
		this._logArea.append(msg);
	}
};
