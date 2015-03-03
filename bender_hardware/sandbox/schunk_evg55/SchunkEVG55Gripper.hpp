#ifndef _SCHUNKEVG55GRIPPER_HPP
#define _SCHUNKEVG55GRIPPER_HPP

#include "Gripper.hpp"



class SchunkEVG55Gripper: public Gripper {
public:
	//! Defines max gripper opening
	static const float maxOpening = 100.0; // mm
	
public:
	/**
	 * @brief Constructor.
	 */
	SchunkEVG55Gripper();
	
	/**
	 * @brief Destructor.
	 */
	virtual ~SchunkEVG55Gripper();
	
	// inherited from Gripper
	virtual bool connect(SerialPort* port);

	virtual bool isConnected() const;

	virtual void disconnect();
	
	virtual bool clearError();

	virtual bool home();
	
	virtual bool isReferenced() const;

	virtual bool open();

	virtual bool close();

	virtual void stop();

	virtual double getConfiguration() const;

	virtual bool setConfiguration(double q);

	virtual unsigned getStatus() const;
	
protected:

private:
	SerialPort* _port;
	unsigned char _moduleId;
	bool _connected;
	bool _referenced;
};

#endif // _SCHUNKEVG55GRIPPER_HPP
