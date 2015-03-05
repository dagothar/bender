#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <serial/RWHWSerialPort.hpp>

using namespace std;
using namespace evg55::serial;



int main() {
	// create a new port
	SerialPort* port = new RWHWSerialPort();
	port->open("/dev/ttyUSB0", 9600);
	
	/* TEST CODE */
	
	/* /TEST CODE */
	
	port->close();
	delete port;
	
	return 0;
}
