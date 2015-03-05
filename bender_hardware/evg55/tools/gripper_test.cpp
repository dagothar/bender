#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <serial/RWHWSerialPort.hpp>
#include <gripper/EVG55.hpp>
#include <mcsprotocol/MCSProtocol.hpp>

using namespace std;
using namespace evg55::serial;
using namespace evg55::gripper;
using namespace evg55::mcsprotocol;

int main() {
	// create a new port
	SerialPort* port = new RWHWSerialPort();
	port->open("/dev/ttyUSB0", 9600);
	
	// create gripper
	EVG55 gripper;
	gripper.connect(port, 0x0c);
	
	/* TEST CODE */
	gripper.home();
	sleep(10);
	
	/* /TEST CODE */
	
	port->close();
	delete port;
	
	return 0;
}
