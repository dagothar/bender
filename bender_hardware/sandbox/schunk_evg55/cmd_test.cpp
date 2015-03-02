#include <iostream>
#include <sstream>
#include <iomanip>
#include <unistd.h>
#include <boost/lexical_cast.hpp>
#include "RWHWSerialPort.hpp"
#include "SchunkEVG55Gripper.hpp"
#include "MCSProtocol.hpp"
#include "SchunkCRC16.hpp"

using namespace std;



int main(int argc, char* argv[]) {
	// open port
	SerialPort* port = new RWHWSerialPort();
	port->open("/dev/ttyUSB0", 9600);
	port->clean();
	
	// send a packet specified in the cmd line
	typedef std::vector<unsigned char> Packet;
	Packet data;
	
	for (int i = 1; i < argc; ++i) {
		stringstream sstr;
		sstr << hex << argv[i];
		
		unsigned int byte;
		sstr >> byte;
		
		data.push_back(static_cast<unsigned char>(byte));
	}
	
	// create a packet & append crc sum
	Packet packet;
	packet.push_back('\x05');
	packet.insert(packet.end(), data.begin(), data.end());
	CRC* crcCalculator = new SchunkCRC16();
	uint16_t crcSum = crcCalculator->crc(packet);
	delete crcCalculator;

	char* crcBytes = reinterpret_cast<char*>(&crcSum);
	packet.push_back(crcBytes[0]);
	packet.push_back(crcBytes[1]);
	
	// send packet
	cout << "Sending: " << endl;
	MCSProtocol::printPacket(packet);
	
	MCSProtocol::send(port, packet);
	
	//sleep(5);
	
	// receive answer
	char buf[30];
	
	for (int i = 0; i < 30; ++i) {
		buf[i] = 0;
	}
	
	cout << port->read(buf, 3, 200) << endl; // blocking
	port->read(buf+3, (unsigned char)buf[2], 200);
	
	unsigned length = 3 + (unsigned char)buf[2];
	cout << length << endl;
	
	for (int i = 0; i < length; ++i) {
		cout << hex << setw(2) << +(unsigned char)buf[i] << ' ';
	}
	
	// print out answer
	
	// close port
	port->clean();
	port->close();
	delete port;
	
	return 0;
}
