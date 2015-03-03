#include "SchunkEVG55Gripper.hpp"
#include "SerialPort.hpp"
#include "MCSProtocol.hpp"
#include <iostream>

using namespace std;

SchunkEVG55Gripper::SchunkEVG55Gripper() :
	_connected(false)
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

bool SchunkEVG55Gripper::clearError() {
	if (!_connected) {
		cerr << "home(): The gripper is not connected!" << endl;
		
		return false;
	}
	
	
}

bool SchunkEVG55Gripper::home() {
	if (!_connected) {
		cerr << "home(): The gripper is not connected!" << endl;
		
		return false;
	}
	
	_referenced = MCSProtocol::homeCmd(_port, _moduleId);
	
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
	
	return MCSProtocol::moveCurrentCmd(_port, _moduleId, 0.5);
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
	
	return MCSProtocol::moveCurrentCmd(_port, _moduleId, -0.5);
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
	
	return MCSProtocol::movePositionCmd(_port, _moduleId, q);
}

unsigned SchunkEVG55Gripper::getStatus() const {
}
