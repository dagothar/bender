#include <iostream>
#include <iomanip>
#include <unistd.h>
#include "RWHWSerialPort.hpp"
#include "SchunkEVG55Gripper.hpp"
#include "MCSProtocol.hpp"

using namespace std;



int main() {
	// create a new port
	SerialPort* port = new RWHWSerialPort();
	port->open("/dev/ttyUSB0", 9600);
	
	// create the gripper
	Gripper* gripper = new SchunkEVG55Gripper();
	
	gripper->connect(port);
	cout << gripper->isConnected() << endl;
	
	// reference gripper
	cout << gripper->home() << endl;
	//sleep(2);
	
	// move gripper
	cout << gripper->setConfiguration(35.0) << endl;
	cout << gripper->close() << endl;
	
	//cout << gripper->close() << endl;
	
	delete gripper;
	
	port->close();
	delete port;
	
	return 0;
}
