/**
 * @author Adam Wolniakowski
 * @version 1.2
 * @since 25-08-2014
 */

package pl.dagothar.template;



import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import javax.swing.*;



/**
 * Main application window class
 */
public class MainMenu extends JMenuBar {
	/**
	 * Default constructor
	 */
	public MainMenu() {
		super();

		initialize();
	}
	
	/**
	 * Initializes menu bar
	 */
	private void initialize() {
		// create File menu
		JMenu fileMenu = new JMenu("File");
		fileMenu.setMnemonic(KeyEvent.VK_F);
		
		ImageIcon newIcon = new ImageIcon(ClassLoader.getSystemResource("images/new.png"));
		JMenuItem newItem = new JMenuItem("New", newIcon);
		newItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_N, ActionEvent.CTRL_MASK));
		newItem.setMnemonic(KeyEvent.VK_N);
		fileMenu.add(newItem);
		
		JMenuItem openItem = new JMenuItem("Open");
		openItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_O, ActionEvent.CTRL_MASK));
		openItem.setMnemonic(KeyEvent.VK_O);
		fileMenu.add(openItem);
		
		ImageIcon saveIcon = new ImageIcon(ClassLoader.getSystemResource("images/save.png"));
		JMenuItem saveItem = new JMenuItem("Save", saveIcon);
		saveItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_S, ActionEvent.CTRL_MASK));
		saveItem.setMnemonic(KeyEvent.VK_S);
		fileMenu.add(saveItem);
		
		fileMenu.addSeparator();
		
		ImageIcon exitIcon = new ImageIcon(ClassLoader.getSystemResource("images/exit.png"));
		JMenuItem exitItem = new JMenuItem("Exit", exitIcon);
		exitItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_Q, ActionEvent.CTRL_MASK));
		exitItem.setMnemonic(KeyEvent.VK_X);
		exitItem.addActionListener(
			new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					System.exit(0);
				}
			}
		);
		fileMenu.add(exitItem);
		
		add(fileMenu);
		
		// create Edit menu
		JMenu editMenu = new JMenu("Edit");
		editMenu.setMnemonic(KeyEvent.VK_E);
		
		JMenuItem undoItem = new JMenuItem("Undo");
		editMenu.add(undoItem);
		
		editMenu.addSeparator();
		
		JMenuItem cutItem = new JMenuItem("Cut");
		editMenu.add(cutItem);
		
		JMenuItem copyItem = new JMenuItem("Copy");
		editMenu.add(copyItem);
		
		JMenuItem pasteItem = new JMenuItem("Paste");
		editMenu.add(pasteItem);
		
		add(editMenu);
		
		// create Help menu
		JMenu helpMenu = new JMenu("Help");
		helpMenu.setMnemonic(KeyEvent.VK_H);
		
		JMenuItem helpItem = new JMenuItem("Help");
		helpItem.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_H, ActionEvent.CTRL_MASK));
		helpItem.setMnemonic(KeyEvent.VK_H);
		helpItem.addActionListener(
			new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					help();
				}
			}
		);
		helpMenu.add(helpItem);
		
		helpMenu.addSeparator();
		
		JMenuItem aboutItem = new JMenuItem("About");
		aboutItem.setMnemonic(KeyEvent.VK_A);
		aboutItem.addActionListener(
			new ActionListener() {
				@Override
				public void actionPerformed(ActionEvent e) {
					about();
				}
			}
		);
		helpMenu.add(aboutItem);
		
		add(helpMenu);
	}
	
	/**
	 * Displays help
	 */
	public void help() {
	}
	
	/**
	 * Displays about dialog
	 */
	public void about() {
		JOptionPane.showMessageDialog(getParent(), "JAVA project template", "About", JOptionPane.INFORMATION_MESSAGE);
	}
};
