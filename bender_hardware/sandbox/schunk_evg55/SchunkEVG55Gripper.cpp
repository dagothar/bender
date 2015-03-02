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
}

void SchunkEVG55Gripper::disconnect() {
}

bool SchunkEVG55Gripper::home() {
	if (!_connected) return false;
	
	return MCSProtocol::homeCmd(_port, _moduleId);
}

bool SchunkEVG55Gripper::open() {
	if (!_connected) return false;
	
	return setConfiguration(maxOpening);
}

bool SchunkEVG55Gripper::close() {
	if (!_connected) return false;
	
	return setConfiguration(0.0);
}

void SchunkEVG55Gripper::stop() {
}

double SchunkEVG55Gripper::getConfiguration() const {
}

bool SchunkEVG55Gripper::setConfiguration(double q) {
	if (!_connected) return false;
	
	return MCSProtocol::movePositionCmd(_port, _moduleId, q);
}

unsigned SchunkEVG55Gripper::getStatus() const {
}
