#include "EVG55.hpp"

#include <iostream>
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
	_connected(false)
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
	
	// set state sending interval to 0.1 s
	Command cmd = CommandFactory::makeGetStateCommand(_id, 0.1);
	MCSProtocol::send(_port, cmd);
	
	boost::lock_guard<boost::mutex> guard(_mtx);
	_connected = true;
	
	// launch listening thread
	boost::function<void(void)> f = boost::bind(&EVG55::listenerFunc, this);
	_listenerThread = boost::thread(f);
	
	return _connected;
}

void EVG55::disconnect() {
	// reset state sending interval
	Command cmd = CommandFactory::makeGetStateCommand(_id, 0.0);
	MCSProtocol::send(_port, cmd);
	
	_connected = false;
	_listenerThread.join();
}

bool EVG55::isConnected() const {
	boost::lock_guard<boost::mutex> guard(_mtx);
	
	return _connected;
}

void EVG55::home() {
	Command refCmd = CommandFactory::makeReferenceCommand(_id);
	MCSProtocol::send(_port, refCmd);
}

void EVG55::listenerFunc() {
	const int maxMissedCount = 100;
	
	int missedCount = 0;
	
	while (isConnected()) {
		usleep(1);
		
		// try to read a new response
		Response response;
		bool responseOk = MCSProtocol::receive(_port, response);
		
		// check if still connected
		if (responseOk) {
			missedCount = 0;
		} else {
			++missedCount;
			
			if (missedCount > maxMissedCount) {
				boost::lock_guard<boost::mutex> guard(_mtx);
				_connected = false;
				continue;
			}
		}
		
		// decode message
		ByteVector state = response.getData();
		//float position = DataConversion::byteV
		
		cout << response << endl;
	}
}
