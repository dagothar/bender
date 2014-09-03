/**
 * @author Adam Wolniakowski
 */

package pl.dagothar.grippers;



import java.awt.*;
import java.awt.event.*;
import javax.swing.*;



/**
 * Main application window class
 */
public class MainMenu extends JMenuBar implements ActionListener {
	/**
	 * Constructor
	 */
	public MainMenu() {
		super();
		
		initialize();
	}
	
	/**
	 * Build menu
	 */
	private void initialize() {
		// create File menu
		JMenu fileMenu = new JMenu("File");
		JMenuItem newItem = new JMenuItem("New", KeyEvent.VK_N);
		newItem.setActionCommand("new");
		newItem.addActionListener(this);
		fileMenu.add(newItem);
		JMenuItem openItem = new JMenuItem("Open", KeyEvent.VK_O);
		openItem.addActionListener(this);
		fileMenu.add(openItem);
		JMenuItem saveItem = new JMenuItem("Save", KeyEvent.VK_S);
		saveItem.addActionListener(this);
		fileMenu.add(saveItem);
		fileMenu.addSeparator();
		JMenuItem exitItem = new JMenuItem("Exit", KeyEvent.VK_X);
		exitItem.setActionCommand("exit");
		exitItem.addActionListener(this);
		fileMenu.add(exitItem);
		add(fileMenu);
		// create Edit menu
		JMenu editMenu = new JMenu("Edit");
		add(editMenu);
		// create Evaluation menu
		JMenu evaluationMenu = new JMenu("Evaluation");
		add(evaluationMenu);
		// create Help menu
		JMenu helpMenu = new JMenu("Help");
		JMenuItem aboutItem = new JMenuItem("About", KeyEvent.VK_A);
		aboutItem.setActionCommand("about");
		aboutItem.addActionListener(this);
		helpMenu.add(aboutItem);
		add(helpMenu);
		
	}
	
	/**
	 * Listen for menu events
	 */
	public void actionPerformed(ActionEvent e) {
		if ("exit".equals(e.getActionCommand())) {
			MainWindow.getInstance().dispose();
		}
		else if ("new".equals(e.getActionCommand())) {
			MainWindow.getInstance().newGripper();
		}
		else if ("about".equals(e.getActionCommand())) {
			// create short dialog w/ information
			MainWindow.getInstance().message("Executing 'about' action.\n");
			JOptionPane.showMessageDialog(null, "Hi. This is a gripper evaluation software!");
		}
	}
};

