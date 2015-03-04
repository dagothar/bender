#ifndef _EVG55_HPP
#define _EVG55_HPP



class EVG55 {
public:
	//! Status flags.
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
	
	//! Error codes.
	enum Error {
		ErrorSoftLow = 0x0d5
	};
	
public:
	/**
	 * @brief Constructor.
	 */
	EVG55();
	
	/**
	 * @brief Destructor.
	 */
	virtual ~EVG55();
	
protected:

private:
	/*SerialPort* _port;
	unsigned char _moduleId;
	volatile unsigned short _status;
	volatile bool _connected;
	volatile bool _referenced;
	volatile bool _error;
	volatile unsigned short _errorCode;
	mutable volatile bool _newInfo; // set by status thread when receiving new info
	volatile float _position;
	
	mutable boost::mutex _statusMutex;
	boost::thread _statusThread;*/
};

#endif // _EVG55_HPP
