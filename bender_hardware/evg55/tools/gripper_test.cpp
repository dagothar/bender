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
	gripper->home();
	cout << gripper->getStatus() << " " << gripper->getConfiguration() << endl;
	
	// move gripper
	gripper->setConfiguration(35.0);
	cout << gripper->getStatus() << " " << gripper->getConfiguration() << endl;
	
	gripper->setConfiguration(15.0);
	cout << gripper->getStatus() << " " << gripper->getConfiguration() << endl;
	
	gripper->setConfiguration(25.0);
	cout << gripper->getStatus() << " " << gripper->getConfiguration() << endl;
	
	gripper->close();
	cout << gripper->getStatus() << " " << gripper->getConfiguration() << endl;
	
	delete gripper;
	
	port->close();
	delete port;
	
	return 0;
}
