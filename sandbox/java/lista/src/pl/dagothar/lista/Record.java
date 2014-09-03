/**
 * @author Adam Wolniakowski
 * @version 1.2
 * @since 25-08-2014
 */

package pl.dagothar.lista;



/**
 * Dummy class for storing data
 */
public class Record {
	private String _name = "";
	private int _id = 0;
	
	public Record(String name, int id) {
		_name = name;
		_id = id;
	}
	
	public String getName() {
		return _name;
	}
	
	public int getId() {
		return _id;
	}
};
