#ifndef _SCHUNKEVG55GRIPPER_HPP
#define _SCHUNKEVG55GRIPPER_HPP

#include "Gripper.hpp"

#include <boost/thread.hpp>



class SchunkEVG55Gripper: public Gripper {
public:
	//! Encoding of the status flags.
	enum Status {
		StatusReferenced = 0x01,
		StatusMoving = 0x02,
		StatusProgramMode = 0x04,
		StatusWarning = 0x08,
		StatusError = 0x10,
		StatusBrake = 0x20,
		StatusMoveEnd = 0x40,
		StatusPositionReached = 0x80
	};
	
	enum Error {
		ErrorSoftLow = 0x0d5
	};
	
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
	
	virtual unsigned short getError() const;
	
	virtual bool clearError();

	virtual bool home();
	
	virtual bool isReferenced() const;

	virtual bool open();

	virtual bool close();

	virtual void stop();

	virtual double getConfiguration() const;

	virtual bool setConfiguration(double q);

	virtual unsigned short getStatus() const;
	
protected:
	/**
	 * @brief A function for the thread listening to gripper messages.
	 */
	void statusThreadFunc(SerialPort* port, unsigned id);
	
	//! Waits till new information received from module.
	void waitForNewInfo() const;
	
	//! Return \b true if position reached.
	bool isPositionReached() const;

private:
	SerialPort* _port;
	unsigned char _moduleId;
	volatile unsigned short _status;
	volatile bool _connected;
	volatile bool _referenced;
	volatile bool _error;
	volatile unsigned short _errorCode;
	mutable volatile bool _newInfo; // set by status thread when receiving new info
	volatile float _position;
	
	mutable boost::mutex _statusMutex;
	boost::thread _statusThread;
};

#endif // _SCHUNKEVG55GRIPPER_HPP
