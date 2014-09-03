/**
 * @author Adam Wolniakowski
 */

package pl.dagothar.grippers;



import java.awt.EventQueue;



/**
 * Main application class
 */
public class Main {
	/**
	 * Default constructor
	 */
	public Main() {
	}
	
	/**
	 * Main
	 */
	public static void main(String args[]) {
		java.awt.EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					MainWindow mainWindow = MainWindow.getInstance();
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
		
	}
};
