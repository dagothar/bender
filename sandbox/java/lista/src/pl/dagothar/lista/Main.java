/**
 * @author Adam Wolniakowski
 */

package pl.dagothar.lista;



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
					MainWindow mainWindow = new MainWindow("Template");
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
		
	}
};
