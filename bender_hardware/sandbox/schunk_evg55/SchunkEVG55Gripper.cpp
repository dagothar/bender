#include "SchunkEVG55Gripper.hpp"
#include "SerialPort.hpp"
#include "MCSProtocol.hpp"
#include <iostream>
#include <unistd.h>

using namespace std;

SchunkEVG55Gripper::SchunkEVG55Gripper() :
	_connected(false),
	_referenced(false),
	_error(false)
{
}

SchunkEVG55Gripper::~SchunkEVG55Gripper() {
}

bool SchunkEVG55Gripper::connect(SerialPort* port) {
	_port = port;
	
	// try all the possible module IDs
	for (unsigned i = 0; i < 256; ++i) {
		if (MCSProtocol::ping(_port, (char) i)) {
			_moduleId = (char) i;
			_connected = true;
			
			cout << "ID=" << (int)_moduleId << endl; // TODO remove output
			
			return true;
		}
	}
	
	_connected = false;
	return false;
}

bool SchunkEVG55Gripper::isConnected() const {
	return _connected;
}

void SchunkEVG55Gripper::disconnect() {
}

bool SchunkEVG55Gripper::isError() const {
	return _error;
}

bool SchunkEVG55Gripper::clearError() {
	if (!_connected) {
		cerr << "home(): The gripper is not connected!" << endl;
		
		return false;
	}
	
	
}

bool SchunkEVG55Gripper::home() {
	_referenced = false;
	
	if (!_connected) {
		cerr << "home(): The gripper is not connected!" << endl;
		
		return false;
	}
	
	if (_error) {
		cerr << "home(): The gripper is in error state!" << endl;
		
		return false;
	}
	
	if (!MCSProtocol::send(_port, MCSProtocol::makeReferenceCommand(_moduleId))) {
		cerr << "home(): Sending reference command failed!" << endl;
		
		return false;
	}
	
	MCSProtocol::Message response;
	if (!MCSProtocol::receive(_port, response)) {
		cerr << "home(): No response from the reference command!" << endl;
		
		return false;
	}
	
	if (response.messageType == MCSProtocol::MessageOk) {
		_referenced = true;
		sleep(3);
	}
	else if (response.messageType == MCSProtocol::MessageError) {
		_error = true;
		_referenced = false;
	}
	
	return _referenced;
}

bool SchunkEVG55Gripper::isReferenced() const {
	if (!_connected) {
		cerr << "isReferenced(): The gripper is not connected!" << endl;
		
		return false;
	}
	
	return _referenced;
}

bool SchunkEVG55Gripper::open() {
	if (!_connected) {
		cerr << "open(): The gripper is not connected!" << endl;
		
		return false;
	}
	
	if (!_referenced) {
		cerr << "open(): The gripper is not referenced!" << endl;
		
		return false;
	}
	
	//return MCSProtocol::moveCurrentCmd(_port, _moduleId, 0.5);
}

bool SchunkEVG55Gripper::close() {
	if (!_connected) {
		cerr << "close(): The gripper is not connected!" << endl;
		
		return false;
	}
	
	if (!_referenced) {
		cerr << "close(): The gripper is not referenced!" << endl;
		
		return false;
	}
	
	if (_error) {
		cerr << "close(): The gripper is in error state!" << endl;
		
		return false;
	}
	
	if (!MCSProtocol::send(_port, MCSProtocol::makeMoveCurrentCommand(_moduleId, 0.1))) {
		cerr << "close(): Sending reference command failed!" << endl;
		
		return false;
	}
	
	int count = 0;
	while (count < 100) {
		usleep(1);
		++count;
		
		MCSProtocol::Message response;
		if (!MCSProtocol::receive(_port, response)) {
			continue;
		}
		
		if (response.messageType == MCSProtocol::MessageMoveBlocked) {
			return true;
		}
		else if (response.messageType == MCSProtocol::MessageError) {
			_error = true;
			return false;
		}
	}
	
	return false;
}

void SchunkEVG55Gripper::stop() {
}

double SchunkEVG55Gripper::getConfiguration() const {
}

bool SchunkEVG55Gripper::setConfiguration(double q) {
	if (!_connected) {
		cerr << "setConfiguration(): The gripper is not connected!" << endl;
		
		return false;
	}
	
	if (!_referenced) {
		cerr << "setConfiguration(): The gripper is not referenced!" << endl;
		
		return false;
	}
	
	if (_error) {
		cerr << "setConfiguration(): The gripper is in error state!" << endl;
		
		return false;
	}
	
	if (!MCSProtocol::send(_port, MCSProtocol::makeMovePositionCommand(_moduleId, q))) {
		cerr << "setConfiguration(): Sending reference command failed!" << endl;
		
		return false;
	}
	
	int count = 0;
	while (count < 100) {
		usleep(1);
		++count;
		
		MCSProtocol::Message response;
		if (!MCSProtocol::receive(_port, response)) {
			continue;
		}
		
		if (response.messageType == MCSProtocol::MessagePositionReached) {
			return true;
		}
		else if (response.messageType == MCSProtocol::MessageError) {
			_error = true;
			return false;
		}
	}
	
	return false;
}

unsigned SchunkEVG55Gripper::getStatus() const {
}
