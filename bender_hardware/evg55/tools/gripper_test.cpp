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
	if (!gripper.connect(port, 0x0c)) {
		cout << "Cannot connect to the gripper module" << endl;
	}
	
	/* TEST CODE */
	// check for error and clear if any
	if (!gripper.isOk(true)) {
		cout << "Gripper error " << +gripper.getErrorCode() << " - trying to clear..." << endl;
		
		if (!gripper.clearError()) {
			cout << "Error occured" << endl;
		
			port->close();
			delete port;
			
			return -1;
		}
	}
	
	// test referencing
	cout << "Referencing: " << endl;
	if (!gripper.home()) {
		cout << "Error occured" << endl;
		
		port->close();
		delete port;
		
		return -1;
	} else {
		cout << "succesful" << endl;
	}
	
	// test move to position
	cout << "Moving: " << endl;
	gripper.move(10.0);
	cout << gripper.getPosition(true) << endl;
	
	gripper.move(20.0);
	cout << gripper.getPosition(true) << endl;
	
	sleep(1);
	
	// test closing
	cout << "Closing: " << endl;
	//gripper.close();
	
	cout << hex << +gripper.getErrorCode(true) << endl;
	
	/* /TEST CODE */
	
	port->close();
	delete port;
	
	return 0;
}
