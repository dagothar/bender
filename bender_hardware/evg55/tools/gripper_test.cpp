#include <iostream>
#include <iomanip>
#include <sstream>
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
	
	if (!gripper.isOk()) gripper.clearError();
	
	/* TEST CODE */
	// check for error and clear if any
	if (!gripper.isOk()) {
		cout << "Gripper error, trying to clear..." << endl;
		
		gripper.clearError();
		
		if (!gripper.isOk()) {
			cout << "Error occured" << endl;
		
			port->close();
			delete port;
			
			return -1;
		}
	}
	
	// test referencing
	do {
		cout << "Referencing..." << endl;
		
		gripper.home();
		sleep(1);
		
	} while (gripper.isOk() && !gripper.isReferenced());
	
	if (!gripper.isOk(false)) {
		cout << "Error occured" << endl;
		
		port->close();
		delete port;
		
		return -1;
	}
	
	// test move to position
	gripper.move(10.0);
	cout << gripper.getPosition(true) << endl;
	
	gripper.move(20.0);
	cout << gripper.getPosition(true) << endl;
	
	sleep(1);
	gripper.close();
	sleep(1);
	
	cout << hex << +gripper.getErrorCode(true) << endl;
	
	/* /TEST CODE */
	
	port->close();
	delete port;
	
	return 0;
}
