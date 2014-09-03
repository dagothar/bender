/**
 * @author Adam Wolniakowski
 * @version 1.0
 * @since 14-08-2014
 */

package pl.dagothar.grippers;



/**
 * Provides a gripper builder
 */
public class GripperBuilder {
	private String _name = "gripper";
	private double[] _params = {
		0,
		0.1, 0.02, 0.01, // general dimensions
		0.0, 0.0, // chamfer
		0.0,
		0.0, 0.0, // cut
		0.0,
		0.075, // tcp position
		0.05, // opening
		0.05, // stroke
		25 // force
	};
	
	/**
	 * Constructor
	 */
	public GripperBuilder() { }
	
	/**
	 * Creates an instance of Gripper
	 */
	public Gripper buildGripper() {
		Gripper gripper = new Gripper();
		
		gripper.setName(this._name);
		gripper.setParameters(this._params);
		
		return gripper;
	}
	
	public GripperBuilder name(String name) {
		this._name = name;
		return this;
	}
	
	/**
	 * Sets gripper general dimensions
	 * @param length, width, depth [in] in meters
	 */
	public GripperBuilder dimensions(double length, double width, double depth) {
		this._params[1] = length;
		this._params[2] = width;
		this._params[3] = depth;
		return this;
	}
	
	/**
	 * Sets gripper chamfering.
	 * @param depth [in] depth in 0-1
	 * @param angle [in] angle of the chamfering in degrees (0-90)
	 */
	public GripperBuilder chamfering(double depth, double angle) {
		this._params[4] = depth;
		this._params[5] = angle;
		return this;
	}
	
	/**
	 * Sets gripper cutout
	 * @param depth [in] in meters
	 * @param angle [in] in degrees
	 */
	public GripperBuilder cutout(double depth, double angle) {
		this._params[7] = depth;
		this._params[8] = angle;
		return this;
	}
	
	/**
	 * Sets gripper TCP position (from the base)
	 */
	public GripperBuilder tcp(double position) {
		this._params[10] = position;
		return this;
	}
	
	/**
	 * Sets gripper opening
	 */
	public GripperBuilder opening(double opening) {
		this._params[11] = opening;
		return this;
	}
	
	/**
	 * Sets gripper stroke
	 */
	public GripperBuilder stroke(double stroke) {
		this._params[12] = stroke;
		return this;
	}
	
	/**
	 * Sets gripper force
	 */
	public GripperBuilder force(double force) {
		this._params[13] = force;
		return this;
	}
};
