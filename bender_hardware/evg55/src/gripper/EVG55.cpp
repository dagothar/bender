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
	
	_connected = true;
	
	return _connected;
}

void EVG55::disconnect() {
	_connected = false;
}

bool EVG55::isConnected() const {
	return _connected;
}

float EVG55::getPosition() const {
	return _position;
}

void EVG55::home() {
	Command refCmd = CommandFactory::makeReferenceCommand(_id);
	MCSProtocol::send(_port, refCmd);
}

/*void EVG55::listenerFunc() {
	const int maxMissedCount = 100;
	
	int missedCount = 0;
	
	while (isConnected()) {
		//usleep(1);
		
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
		float position = DataConversion::byteVector2float(ByteVector(state.begin(), state.begin() + 4));
		
		{
			boost::lock_guard<boost::mutex> guard(_mtx);
			
			_position = position;
		}
		
		//cout << response << endl;
	}
}*/
