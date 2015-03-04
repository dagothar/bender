#include <iostream>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include "RWHWSerialPort.hpp"
#include "SchunkEVG55Gripper.hpp"
#include "MCSProtocol.hpp"
#include "SchunkCRC16.hpp"

using namespace std;



typedef std::vector<unsigned char> Packet;



/*
 * STATUS THREAD
 * 
 * Receives and prints out messages from the gripper
 */
void statusThreadFunc(SerialPort* port) {
	bool connected = true;
	
	while (connected) {
		// read another packet
		const int BUF_LEN = 30;
	
		// read BUF_LEN characters from the serial port
		char buf[BUF_LEN];
		
		// clean buffer
		for (int i = 0; i < BUF_LEN; buf[i++] = 0);
		
		bool received = port->read(buf, BUF_LEN, 200);
		port->clean();
		
		if (!buf[0]) continue;
		
		// calculate message length using dlen
		int msgLength = 3 + static_cast<unsigned char>(buf[2]);
		
		Packet response;
		for (unsigned i = 0; i < msgLength; ++i) {
			response.push_back(buf[i]);
		}
		
		cout << "Received: ";
		MCSProtocol::printPacket(response);
	}
}



int main(int argc, char* argv[]) {
	// connect to a port
	SerialPort* port = new RWHWSerialPort();
	port->open("/dev/ttyUSB0", 9600);
	port->clean();
	
	// launch listening thread
	boost::thread statusThread(statusThreadFunc, port);
	
	/*
	 * MAIN LOOP
	 * 
	 * Asks user for hex commands to send to the gripper
	 */
	bool exit = false;
	while (!exit) {
		string commands;
		
		// ask for user input
		cout << "> ";
		getline(cin, commands);
		
		if (commands[0] == 'q') {
			exit = true;
			continue;
		}
		
		// convert commands to hex values
		stringstream sstr(commands);
		Packet packet;
		
		unsigned int byte;
		while (sstr >> hex >> byte) {
			packet.push_back(static_cast<unsigned char>(byte));
		}
		
		// create a packet and send to the port
		CRC* crcCalculator = new SchunkCRC16();
		uint16_t crcSum = crcCalculator->crc(packet);
		delete crcCalculator;

		char* crcBytes = reinterpret_cast<char*>(&crcSum);
		packet.push_back(crcBytes[0]);
		packet.push_back(crcBytes[1]);
		
		// send packet
		//cout << "Sending: " << endl;
		//MCSProtocol::printPacket(packet);
		
		MCSProtocol::send(port, packet);
	}
	
	port->clean();
	port->close();
	delete port;	
	 
	return 0;
}
