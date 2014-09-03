/**
 * Implements a gripper description.
 * 
 * @author Adam Wolniakowski
 * @version 1.0
 * @since 14-08-2014
 */

package pl.dagothar.grippers;



/**
 * Provides a gripper description
 */
public class Gripper {
	private String _name;
	
	// parametrization
	private double _length;
	private double _width;
	private double _depth;
	private double _chamferDepth;
	private double _chamferAngle;
	private double _cutDepth;
	private double _cutAngle;
	private double _tcpPosition;
	private double _opening;
	private double _stroke;
	private double _force;
	
	// quality metric
	private double _success;
	private double _coverage;
	private double _averageWrench;
	private double _topWrench;
	private double _robustness;
	private double _volume;
	private double _maxStress;
	private double _penalty;
	private double _quality;
	
	// Constructors
	/**
	 * Default constructor
	 */
	public Gripper() {
	}
	
	// Accessors
	public String getName() { return this._name; }
	public double getLength() { return this._length; }
	public double getWidth() { return this._width; }
	public double getDepth() { return this._depth; }
	public double getChamferDepth() { return this._chamferDepth; }
	public double getChamferAngle() { return this._chamferAngle; }
	public double getCutDepth() { return this._cutDepth; }
	public double getCutAngle() { return this._cutAngle; }
	public double getTcpPosition() { return this._tcpPosition; }
	public double getOpening() { return this._opening; }
	public double getStroke() { return this._stroke; }
	public double getForce() { return this._force; }
	public double[] getParameters() {
		return new double[]{
			0.0, // cutout type -- required for compatibility with C++ format
			this._length,
			this._width,
			this._depth,
			this._chamferDepth,
			this._chamferAngle,
			0.0, // cutout position -- required for compatibility with C++ format
			this._cutDepth,
			this._cutAngle,
			0.0, // cutout radius -- required for compatibility with C++ format
			this._tcpPosition,
			this._opening,
			this._stroke,
			this._force
		};
	}
	
	public void setName(String name) { this._name = name; }
	public void setLength(double length) { this._length = length; }
	public void setWidth(double width) { this._width = width; }
	public void setDepth(double depth) { this._depth = depth; }
	public void setChamferDepth(double chamferDepth) { this._chamferDepth = chamferDepth; }
	public void setChamferAngle(double chamferAngle) { this._chamferAngle = chamferAngle; }
	public void setCutDepth(double cutDepth) { this._cutDepth = cutDepth; }
	public void setCutAngle(double cutAngle) { this._cutAngle = cutAngle; }
	public void setTcpPosition(double tcpPosition) { this._tcpPosition = tcpPosition; }
	public void setOpening(double opening) { this._opening = opening; }
	public void setStroke(double stroke) { this._stroke = stroke; }
	public void setForce(double force) { this._force = force; }
	public void setParameters(double[] params) {
		this._length = params[1];
		this._width = params[2];
		this._depth = params[3];
		this._chamferDepth = params[4];
		this._chamferAngle = params[5];
		this._cutDepth = params[7];
		this._cutAngle = params[8];
		this._tcpPosition = params[10];
		this._opening = params[11];
		this._stroke = params[12];
		this._force = params[13];
	}
	
	// Other methods
	/**
	 * Provides a conversion to String
	 */
	@Override
	public String toString() {
		return "[" + this._name + "]";
	}
};
