#include <iostream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <evg55/serial/RWHWSerialPort.hpp>
#include <evg55/mcsprotocol/MCSProtocol.hpp>
#include <evg55/mcsprotocol/CommandFactory.hpp>

using namespace std;
using namespace evg55::serial;
using namespace evg55::mcsprotocol;

int main() {
	// create a new port
	SerialPort* port = new RWHWSerialPort();
	port->open("/dev/ttyUSB0", 9600);
	port->clean();
	
	Response response;
	MCSProtocol::emit(port, CommandFactory::makeAcknowledgementCommand(0x0c), response);
	sleep(1);
	
	port->close();
	delete port;
	
	return 0;
}
