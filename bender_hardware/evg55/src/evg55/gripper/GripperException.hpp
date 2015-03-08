#ifndef _GRIPPEREXCEPTION_HPP
#define _GRIPPEREXCEPTION_HPP

#include <stdexcept>
#include <string>
#include <sstream>

namespace evg55 {
namespace gripper {

/**
 * Base class for gripper exceptions.
 */
class GripperException: public std::runtime_error {
public:
	//! Constructor.
	GripperException() : std::runtime_error("") {}
	
	//! Destructor.
	virtual ~GripperException() throw() {}
	
	virtual const char* what() throw() {
		return "Gripper exception";
	}
};



/**
 * Thrown when failed to acknowledge a command
 */
class GripperAckException: public GripperException {
public:
	//! Constructor
	GripperAckException(const std::string& command) :
		_command(command)
	{}
	
	//! Destructor.
	virtual ~GripperAckException() throw() {}
	
	virtual const char* what() throw() {
		return "lol"; //("Failed to acknowledge command: " + _command).c_str();
	}
	
protected:
	const std::string _command;
};



/**
 * Gripper is not referenced exception.
 */
class GripperNotReferenced: public GripperException {
public:
	virtual const char* what() throw() {
		return "Gripper is not referenced";
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

}
}

#endif // _GRIPPEREXCEPTION_HPP
