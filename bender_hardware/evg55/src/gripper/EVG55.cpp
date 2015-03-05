#include "EVG55.hpp"

#include <iostream>
#include <cmath>
#include <boost/bind.hpp>

#include <mcsprotocol/CommandFactory.hpp>
#include <mcsprotocol/MCSProtocol.hpp>
#include <mcsprotocol/DataConversion.hpp>
#include "GripperException.hpp"

using namespace std;
using namespace evg55::serial;
using namespace evg55::gripper;
using namespace evg55::mcsprotocol;

EVG55::EVG55() :
	_port(NULL),
	_id(0),
	_connected(false),
	_ok(true),
	_referenced(false),
	_status(0)
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
	
	_connected = true;
	
	return _connected;
}

void EVG55::disconnect() {
	_connected = false;
}
bool EVG55::poll() {
	const int maxMissedCount = 10;
	
	// send get state request
	_port->clean();
	
	Command getStateCmd = CommandFactory::makeGetStateCommand(_id);
	MCSProtocol::send(_port, getStateCmd);
	Response response;
	
	// wait for proper response
	int missedCount = 0;
	while (missedCount <= maxMissedCount) {
		usleep(100);
		
		if (MCSProtocol::receive(_port, response)) {
			break;
		} else {
			++missedCount;
		}
	}
	
	if (missedCount > maxMissedCount) {
		_connected = false;
		return false;
	}
	
	// decode state message
	ByteVector state = response.getData();
	_position = DataConversion::byteVector2float(ByteVector(state.begin(), state.begin() + 4));
	_velocity = DataConversion::byteVector2float(ByteVector(state.begin() + 4, state.begin() + 8));
	_current = DataConversion::byteVector2float(ByteVector(state.begin() + 8, state.begin() + 12));
	_status = DataConversion::byteVector2unsignedInt(ByteVector(state.begin() + 12, state.begin() + 14));
	
	// decode status
	_referenced = _status & StatusReferenced;
	_ok = !(_status & StatusError);
	_errorCode = (_status & 0xff00) >> 8;

	_connected = true;
	return true;
}

unsigned short EVG55::getStatus(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _status;
}

bool EVG55::isConnected(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _connected;
}

bool EVG55::isOk(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _connected && _ok;
}

bool EVG55::isReferenced(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _referenced;
}

bool EVG55::isMoving(bool doPoll) {
	if (doPoll) {
		poll();
	}
	
	return _status & StatusMoving;
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

void EVG55::home() {
	Command refCmd = CommandFactory::makeReferenceCommand(_id);
	MCSProtocol::send(_port, refCmd);
}

void EVG55::move(float pos) {
	Command refCmd = CommandFactory::makeMovePositionCommand(_id, pos);
	MCSProtocol::send(_port, refCmd);
}

bool EVG55::moveWait(float pos) {
	move(pos);
	
	// wait till in position
	// TODO: add timeout && error checking
	while (isOk() && fabs(getPosition() - pos) > 1.0) {
		usleep(100);
		
		if (_status & StatusMoveEnd) break;
	}
	
	// check if at destination
	if (isOk() && fabs(getPosition() - pos) < 1.0) {
		return true;
	}
	
	return false;
}
