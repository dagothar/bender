#ifndef _GRIPPEREXCEPTION_HPP
#define _GRIPPEREXCEPTION_HPP

#include <stdexcept>
#include <sstream>



/**
 * Base class for gripper exceptions.
 */
class GripperException: public std::runtime_error {
public:
	//! Constructor.
	GripperException() : std::runtime_error("") {}
	
	virtual const char* what() throw() {
		return "Gripper exception!";
	}
};



/**
 * Gripper is not connected exception.
 */
class GripperNotConnected: public GripperException {
public:
	virtual const char* what() throw() {
		return "Gripper is not connected!";
	}
};



/**
 * Gripper is not referenced exception.
 */
class GripperNotReferenced: public GripperException {
public:
	virtual const char* what() throw() {
		return "Gripper is not referenced!";
	}
};



/**
 * Gripper error exception.
 */
class GripperError: public GripperException {
public:
	//! Constructor.
	GripperError(unsigned short errorCode) : _error(errorCode) {}
	
	//! Destructor.
	virtual ~GripperError() throw() {}
	
	virtual const char* what() throw() {
		std::stringstream sstr;
		sstr << "Gripper error: " <<  _error;
		
		return sstr.str().c_str();
	}
	
private:
	unsigned short _error;
};

#endif // _GRIPPEREXCEPTION_HPP
