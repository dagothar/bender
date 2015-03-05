#include "EVG55.hpp"

#include <iostream>

#include "GripperException.hpp"

using namespace std;

EVG55::EVG55() :
	_port(NULL),
	_id(0)
{
}

EVG55::~EVG55() {
}

bool EVG55::connect(SerialPort* port, unsigned char id) {
	/* Send a trial GetState request and wait for response.
	 * If no response, then connect not succesful */
	
}

unsigned short EVG55::getState() {
}
