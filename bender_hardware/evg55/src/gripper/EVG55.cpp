#include "EVG55.hpp"

#include <iostream>
#include <cmath>
#include <ctime>
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
	
	// toggle impulse messages
	
	_connected = poll();
	
	return _connected;
}

void EVG55::disconnect() {
	_connected = false;
}

bool EVG55::clearError() {
	if (!MCSProtocol::emit(_port, CommandFactory::makeAcknowledgementCommand(_id))) {
		cout << "Failed to send ClearError command" << endl;
		return false;
	}
	
	sleep(1);
	
	poll();
	
	return isOk();
}

bool EVG55::poll() {
	// send get state request	
	Command getStateCmd = CommandFactory::makeGetStateCommand(_id);
	Response response;
	
	// try to getstate several times to deal with nasty impulse commands breaking up communication...
	int count = 0;
	bool received = false;
	do {
		MCSProtocol::send(_port, getStateCmd);
		received = MCSProtocol::ack(_port, getStateCmd, response, 10);
		
		if (!received) {
			if (++count > 10) {
				cout << "Connection lost" << endl;
				_connected = false;
				return false;
			}
		}
	} while (!received);
	
	// decode state message
	ByteVector state = response.getData();
	_position = DataConversion::byteVector2float(ByteVector(state.begin(), state.begin() + 4));
	_velocity = DataConversion::byteVector2float(ByteVector(state.begin() + 4, state.begin() + 8));
	_current = DataConversion::byteVector2float(ByteVector(state.begin() + 8, state.begin() + 12));
	_status = DataConversion::byteVector2unsignedInt(ByteVector(state.begin() + 12, state.begin() + 14));
	
	// decode status
	_referenced = _status & StatusReferenced;
	_errorCode = (_status & 0xff00) >> 8;
	_ok = !(_status & StatusError);

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
	if (!MCSProtocol::emit(_port, CommandFactory::makeReferenceCommand(_id))) {
		cout << "Failed to send Reference command" << endl;
		return false;
	}
	
	clock_t t0 = clock();
	do {
		usleep(1000);
		
		poll();
		
		if (!isOk()) {
			break;
		}
		
		if (_status & StatusReferenced) {
			return true;
		}
	} while (1.0 * (clock() - t0) / CLOCKS_PER_SEC < MoveTimeout);
	
	cout << "Referencing failed" << endl;
	
	return false;
}

bool EVG55::move(float pos) {
	if (pos < 0.0 || pos > MaxOpening) {
		cout << "Position " << pos << " is out of range" << endl;
	}
	
	if (!MCSProtocol::emit(_port, CommandFactory::makeSetTargetVelocityCommand(_id, MaxVelocity))
		|| !MCSProtocol::emit(_port, CommandFactory::makeSetTargetCurrentCommand(_id, MaxCurrent))) {
		
		cout << "Failed to set gripper target velocity or current" << endl;
		return false;
	}
	
	if (!MCSProtocol::emit(_port, CommandFactory::makeMovePositionCommand(_id, pos))) {
		cout << "Failed to send MovePosition command" << endl;
		return false;
	}
	
	// wait till movements complete
	clock_t t0 = clock();
	do {
		usleep(10000);
		
		poll();
		
		if (!isOk()) {
			break;
		}
		
		if (_status & StatusPositionReached || fabs(getPosition() - pos) < EpsPosition) {
			return true;
		}
	} while (1.0 * (clock() - t0) / CLOCKS_PER_SEC < MoveTimeout);
	
	cout << "Move failed" << endl;
	
	return false;
}

bool EVG55::close() {
	if (!MCSProtocol::emit(_port, CommandFactory::makeMoveGripCommand(_id, -MaxCurrent))) {
		cout << "Failed to send MoveGrip command" << endl;
		return false;
	}
	
	// wait till movements complete
	clock_t t0 = clock();
	do {
		usleep(10000);
		
		poll();
		
		cout << "Status" << +_status << endl;
		
		// check if it's the soft limit
		if (_errorCode == 0xd5) {
			cout << "Soft limit reached, clearing error byte" << endl;
			if (!clearError()) {
				break;
			}
			
			continue;
		}
		
		if (!isOk(false)) {
			break;
		}
		
		if (!(_status & StatusMoving)) {
			return true;
		}
	} while (1.0 * (clock() - t0) / CLOCKS_PER_SEC < MoveTimeout);
	
	cout << "Close failed" << endl;
	
	return false;
}
