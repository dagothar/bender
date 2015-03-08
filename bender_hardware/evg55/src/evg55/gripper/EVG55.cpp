#include "EVG55.hpp"

#include <iostream>
#include <cmath>
#include <ctime>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <evg55/mcsprotocol/CommandFactory.hpp>
#include <evg55/mcsprotocol/MCSProtocol.hpp>
#include <evg55/mcsprotocol/DataConversion.hpp>

using namespace std;
using namespace evg55::serial;
using namespace evg55::gripper;
using namespace evg55::mcsprotocol;
using namespace boost::posix_time;

EVG55::EVG55() :
	_port(NULL),
	_id(0),
	_connected(false),
	_ok(true),
	_referenced(false),
	_status(0),
	_errorCode(0)
{
}

EVG55::~EVG55() {
}

bool EVG55::connect(SerialPort* port, unsigned char id) {
	if (_connected) {
		cerr << "Already connected!" << endl;
		return false;
	}
	
	_port = port;
	_id = id;
	
	// poll for the first time
	_port->clean();
	_connected = poll();
	
	return _connected;
}

void EVG55::disconnect() {
	// TODO: what exactly should this do?
	
	_connected = false;
}

bool EVG55::clearError() {
	Response response;
	if (!MCSProtocol::emit(_port, CommandFactory::makeAcknowledgementCommand(_id), response)) {
		cout << "Failed to send ClearError command" << endl;
		return false;
	}
	
	sleep(1);
	
	poll();
	
	return isOk();
}

bool EVG55::poll() {
	/* make get state request */
	Command getStateCmd = CommandFactory::makeGetStateCommand(_id);
	Response response;
	
	/* try to getstate */
	if ( !MCSProtocol::emit(_port, getStateCmd, response, 10) ) {
		return false;
	}
	
	/* decode state message */
	ByteVector state = response.getData();
	_position = DataConversion::byteVector2float(ByteVector(state.begin(), state.begin() + 4));
	_velocity = DataConversion::byteVector2float(ByteVector(state.begin() + 4, state.begin() + 8));
	_current = DataConversion::byteVector2float(ByteVector(state.begin() + 8, state.begin() + 12));
	_status = DataConversion::byteVector2unsignedInt(ByteVector(state.begin() + 12, state.begin() + 14));
	
	/* decode status */
	_referenced = _status & StatusReferenced;
	_errorCode = (_status & 0xff00) >> 8;
	_ok = !(_status & StatusError);

	return true;
}

unsigned short EVG55::getStatus(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _status;
}

bool EVG55::isOk(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _ok;
}

bool EVG55::isReferenced(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _referenced;
}

float EVG55::getPosition(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _position;
}

unsigned char EVG55::getErrorCode(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _errorCode;
}

bool EVG55::home() {
	Response response;
	if (!MCSProtocol::emit(_port, CommandFactory::makeReferenceCommand(_id), response)) {
		throw GripperAckException("Reference");
	}
	
	/* wait till movements complete */
	ptime t0 = microsec_clock::local_time(); // start time
	ptime tend = t0 + seconds(MoveTimeout); // end time
	bool referenced = false;
	
	while (!referenced) {
		/* check for timeout */
		if (microsec_clock::local_time() > tend) {
			cout << "Reference timeout" << endl;
			break;
		}
		 
		usleep(10000);
		poll();
		
		/* break if error occured */
		if (!isOk()) {
			cout << "Reference error" << endl;
			break;
		}
		
		if (_status & StatusReferenced) {
			referenced = true;
		}
	};
	
	return referenced;
}

bool EVG55::move(float pos) {
	/* check if position is sane */
	if (pos < 0.0 || pos > MaxOpening) {
		cout << "Position " << pos << " is out of range" << endl;
	}
	
	/* set limits for velocity and the current */
	Response response;
	if (!MCSProtocol::emit(_port, CommandFactory::makeSetTargetVelocityCommand(_id, MaxVelocity), response)
		|| !MCSProtocol::emit(_port, CommandFactory::makeSetTargetCurrentCommand(_id, MaxCurrent), response)) {
		
		cout << "Failed to set gripper target velocity or current" << endl;
		return false;
	}
	
	/* send position commands */
	if (!MCSProtocol::emit(_port, CommandFactory::makeMovePositionCommand(_id, pos), response)) {
		cout << "Failed to send MovePosition command" << endl;
		return false;
	}
	
	/* wait till movements complete */
	ptime t0 = microsec_clock::local_time(); // start time
	ptime tend = t0 + seconds(MoveTimeout); // end time
	bool reached = false;
	
	while (!reached) {
		/* check for timeout */
		if (microsec_clock::local_time() > tend) {
			cout << "Move timeout" << endl;
			break;
		}
		 
		usleep(10000);
		poll();
		
		/* break if error occured */
		if (!isOk()) {
			cout << "Move error" << endl;
			break;
		}
		
		if (_status & StatusPositionReached || fabs(getPosition() - pos) < EpsPosition) {
			reached = true;
		}
	};
	
	return reached;
}

bool EVG55::close() {
	Response response;
	if (!MCSProtocol::emit(_port, CommandFactory::makeMoveGripCommand(_id, -MaxCurrent), response)) {
		cout << "Failed to send MoveGrip command" << endl;
		return false;
	}
	
	/* wait till movements complete */
	ptime t0 = microsec_clock::local_time(); // start time
	ptime tend = t0 + seconds(MoveTimeout); // end time
	bool closed = false;
	
	while (!closed) {
		/* check for timeout */
		if (microsec_clock::local_time() > tend) {
			cout << "Close timeout" << endl;
			break;
		}
		 
		usleep(10000);
		poll();
		
		cout << "Status: " << _status << endl;
		
		/* check for errors */
		if (!isOk()) {
			/* If gripper is empty, closing ends with module hitting limits.
			 * An error is generated that has to be cleared.
			 */
			if (_errorCode == 0xd5) {
				cout << "Soft limit, attempting to clear error..." << endl;
				
				clearError();
				continue;
			}
			
			cout << "Close error" << endl;
			break;
		}
		
		if (_status & StatusBrake) {
			closed = true;
		}
	};
	
	return closed;
}
