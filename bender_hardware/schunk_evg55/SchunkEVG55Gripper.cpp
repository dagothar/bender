#include "SchunkEVG55Gripper.hpp"

#include <iostream>
#include <unistd.h>
#include <boost/bind.hpp>

#include "SerialPort.hpp"
#include "MCSProtocol.hpp"


using namespace std;

SchunkEVG55Gripper::SchunkEVG55Gripper() :
	_status(0),
	_connected(false),
	_referenced(false),
	_error(false),
	_errorCode(0),
	_newInfo(false),
	_position(0.0)
{
}

SchunkEVG55Gripper::~SchunkEVG55Gripper() {
	disconnect();
}

bool SchunkEVG55Gripper::connect(SerialPort* port) {
	_port = port;
	
	// try all the possible module IDs
	for (unsigned i = 0; i < 256; ++i) {
		if (MCSProtocol::ping(_port, (char) i)) {
			_moduleId = (char) i;
			_connected = true;
			
			cout << "ID=" << (int)_moduleId << endl; // TODO remove output
			
			// launch listening thread
			boost::function<void(SerialPort*, unsigned)> f = boost::bind(&SchunkEVG55Gripper::statusThreadFunc, this, _1, _2);
			_statusThread = boost::thread(f, _port, _moduleId);
			
			//_ok = true;
			
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
	//TODO: how?
	
	// reset getstate interval
	MCSProtocol::send(_port, MCSProtocol::makeGetStateCommand(_moduleId));
	
	_connected = false;
	_statusThread.join();
}

unsigned short SchunkEVG55Gripper::getError() const {
	waitForNewInfo();
	
	_statusMutex.lock();
	unsigned short error = _errorCode;
	_statusMutex.unlock();
	
	return error;
}

bool SchunkEVG55Gripper::clearError() {
	if (!_connected) { throw GripperNotConnected(); }
	
	// TODO: send acknowledge error command
	MCSProtocol::send(_port, MCSProtocol::makeClearErrorCommand(_moduleId));
	
	// wait till error cleared
	while (getError() != 0) {
		usleep(1);
		
		//cout << "Referencing: " << getStatus() << endl;
		
		if (!_connected) { throw GripperNotConnected(); }
	}
	
	return true;
}

bool SchunkEVG55Gripper::home() {
	_referenced = false;
	
	if (!_connected) { throw GripperNotConnected(); }
	
	// send command
	MCSProtocol::send(_port, MCSProtocol::makeReferenceCommand(_moduleId));
	
	// wait till gripper referenced
	while (!isReferenced()) {
		usleep(1);
		
		cout << "Referencing: " << getStatus() << endl;
		
		if (!_connected)  { throw GripperNotConnected(); }
		if (_error)  { throw GripperError(getError()); }
	}
	
	return _referenced;
}

bool SchunkEVG55Gripper::isReferenced() const {
	waitForNewInfo();
	
	_statusMutex.lock();
	bool referenced = _referenced;
	_statusMutex.unlock();
	
	return referenced;
}

bool SchunkEVG55Gripper::isPositionReached() const {
	waitForNewInfo();
	
	_statusMutex.lock();
	bool reached = _status & StatusPositionReached;
	_statusMutex.unlock();
	
	return reached;
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
	if (!_connected)  { throw GripperNotConnected(); }
	
	if (!_referenced)  { throw GripperNotReferenced(); }
	
	if (_error)  { throw GripperError(0); }
	
	MCSProtocol::send(_port, MCSProtocol::makeMoveGripCommand(_moduleId, -5.0));
	
	// wait till move completed
	while (_status & StatusMoving) {
		usleep(1);
		
		if (!_connected)  { throw GripperNotConnected(); }
	
		//if (_error)  { throw GripperError(getError()); }
	}
	
	if (getError() == ErrorSoftLow) { // acknowledge limit
		clearError();
	}
	
	return true;
}

void SchunkEVG55Gripper::stop() {
}

double SchunkEVG55Gripper::getConfiguration() const {
	waitForNewInfo();
	
	_statusMutex.lock();
	float position = _position;
	_statusMutex.unlock();
	
	return position;
}

bool SchunkEVG55Gripper::setConfiguration(double q) {
	if (!_connected)  { throw GripperNotConnected(); }
	
	if (!_referenced)  { throw GripperNotReferenced(); }
	
	if (_error)  { throw GripperError(0); }
	
	MCSProtocol::send(_port, MCSProtocol::makeMovePositionCommand(_moduleId, q));
	
	// wait till move completed
	while (!isPositionReached()) {
		usleep(1);
		
		if (!_connected)  { throw GripperNotConnected(); }
	
		if (_error)  { throw GripperError(0); }
	}
	
	return true;
}

unsigned short SchunkEVG55Gripper::getStatus() const {
	waitForNewInfo();
	
	_statusMutex.lock();
	unsigned short status = _status;
	_statusMutex.unlock();
	
	return status;
}



void SchunkEVG55Gripper::waitForNewInfo() const {
	_statusMutex.lock();
	_newInfo = false;
	_statusMutex.unlock();
	
	bool newInfo;
	do {
		usleep(1);
		
		_statusMutex.lock();
		newInfo = _newInfo;
		_statusMutex.unlock();
	} while (!newInfo);
}



void SchunkEVG55Gripper::statusThreadFunc(SerialPort* port, unsigned id) {
	char* data = new char[14];
	int count = 0;
	
	// send get state command
	MCSProtocol::send(_port, MCSProtocol::makeGetStateCommand(_moduleId, 0.1));
	
	while (_connected) {		
		// receive response
		MCSProtocol::Message response;
		if (!MCSProtocol::receive(_port, response)) {
			++count;
			if (count > 100) _connected = false;
			
			continue;
		}
		
		count = 0;
		
		// decode message & set status flags
		if (response.messageType == MCSProtocol::MessageOther
			&& response.data[MCSProtocol::IndexCommand] == MCSProtocol::CommandGetState
			&& response.data[MCSProtocol::IndexDlen] == 15
			&& response.data[MCSProtocol::IndexModuleId] == id) {
			
			// convert message
			for (int i = 0; i < 14; ++i) {
				data[i] = response.data[MCSProtocol::IndexData+i];
			}

			float position = *(reinterpret_cast<float*>(data));
			unsigned short status = *(reinterpret_cast<unsigned short*>(data+12));
			
			//cout << "Position: " << position << " Status: " << status << endl;
			cout << "thread func" << endl;
			
			// decode message
			_statusMutex.lock();
			
				_newInfo = true;
				_status = status;
				_referenced = status & StatusReferenced;
				_error = status & StatusError;
				_errorCode = (status & 0xff00) >> 8;
				_position = position;
				
			_statusMutex.unlock();
		}
	}
	
	delete data;
}
