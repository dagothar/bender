/**
 * @author Adam Wolniakowski
 * @version 1.2
 * @since 25-08-2014
 */

package pl.dagothar.lista;



import java.util.*;
import java.awt.BorderLayout;
import java.awt.Container;
import javax.swing.*;



/**
 * Main application window class
 */
public class MainWindow extends JFrame {
	private List<Record> _data;
	
	/**
	 * Default constructor
	 */
	public MainWindow(String name) {
		super(name);
		
		_data = new ArrayList<Record>();

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
		
		// add menu
		MainMenu mainMenu = new MainMenu();
		setJMenuBar(mainMenu);
		
		// add scrolled list
		JList list = new JList();
		list.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
		JScrollPane listScroller = new JScrollPane(list);
		contentPane.add(listScroller, BorderLayout.LINE_START);
		
		// add footnote
		JLabel footnoteLabel = new JLabel("dagothar 2014", JLabel.RIGHT);
		contentPane.add(footnoteLabel, BorderLayout.PAGE_END);
		
		setVisible(true);
	}
};
