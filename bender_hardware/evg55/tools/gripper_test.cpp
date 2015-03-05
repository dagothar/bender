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
	// test referencing
	gripper.home();
	
	while (gripper.isOk() && !gripper.isReferenced()) {
		cout << "Referencing..." << endl;
		usleep(100);
	}
	
	if (gripper.getErrorCode() == 0) {
		cout << "Referencing succesful" << endl;
	}
	
	// test move to position
	bool success = gripper.moveWait(10.0);
	gripper.poll();
	cout << "Status: " << gripper.getStatus() << " Position: " << gripper.getPosition() << " success?: " << success << endl;
	
	success = gripper.moveWait(15.0);
	gripper.poll();
	cout << "Status: " << gripper.getStatus() << " Position: " << gripper.getPosition() << " success?: " << success << endl;
	
	success = gripper.moveWait(5.0);
	gripper.poll();
	cout << "Status: " << gripper.getStatus() << " Position: " << gripper.getPosition() << " success?: " << success << endl;
	
	success = gripper.moveWait(75.0);
	gripper.poll();
	cout << "Status: " << gripper.getStatus() << " Position: " << gripper.getPosition() << " success?: " << success << endl;
	
	success = gripper.moveWait(15.0);
	gripper.poll();
	cout << "Status: " << gripper.getStatus() << " Position: " << gripper.getPosition() << " success?: " << success << endl;
	
	gripper.close();
	sleep(2);
	gripper.poll();
	cout << "Status: " << gripper.getStatus() << " Position: " << gripper.getPosition() << " success?: " << success << endl;
	
	cout << +gripper.getErrorCode() << endl;
	//gripper.clearError();
	gripper.poll();
	cout << "Status: " << gripper.getStatus() << " Position: " << gripper.getPosition() << " success?: " << success << endl;
	
	/* /TEST CODE */
	
	port->close();
	delete port;
	
	return 0;
}
