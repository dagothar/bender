#include <iostream>
#include <rw/rw.hpp>
#include "SchunkEVG55.hpp"




using namespace std;
using namespace rwhw;
using namespace rw::common;



int main(int argc, char* argv[])
{
	// create new gripper instance
	SchunkEVG55::Ptr gripper = new SchunkEVG55();
	
	if (!gripper->connectSerial("/dev/ttyUSB0")) {
		return -1;
	}
	
	while (true) {
		unsigned int status = 0;
		gripper->status(status);
		cout << "Gripper status: " << status << endl;
	}
	
	return 0;
}
