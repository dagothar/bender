#include <iostream>
#include <iomanip>
#include <rwhw/serialport/SerialPort.hpp>
#include "SchunkCRC16.hpp"

using namespace std;



int main() {
	//const char msg[] = "\x05\x01\x01\x92\x00";
	
	//cout << msg << endl;
	
	//CRC* crcCalculator = new SchunkCRC16();
	//uint16_t crc = crcCalculator->crc(msg, 4);
	//cout << crc << endl;
	
	//char* bytes = reinterpret_cast<char*>(&crc);
	//cout << hex << int(bytes[0]) << int(bytes[1]) << endl;
	
	// open serial port
	rwhw::SerialPort port;
	bool opened = port.open("/dev/ttyUSB0", rwhw::SerialPort::Baud9600);
	
	if (!opened) {
		cout << "Port is not open!" << endl;
		return 0;
	}
	
	// test module ids
	for (int i = 0; i < 256; ++i) {
		char id = (char) i;
		
		// create message
		vector<unsigned char> msg;
		msg.push_back('\x05'); // MASTER -> SLAVE
		msg.push_back(id); // MODULE ID
		msg.push_back('\x01'); // DLEN
		msg.push_back('\x92'); // GET
		
		// calculate CRC sum of the message
		CRC* crcCalculator = new SchunkCRC16();
		uint16_t crc = crcCalculator->crc(msg);
		
		// apppend crc bytes to the end of the message
		char* crcBytes = reinterpret_cast<char*>(&crc);
		msg.push_back(crcBytes[0]);
		msg.push_back(crcBytes[1]);
		
		// print out the message
		cout << "Sending: " << showbase << internal << hex << setfill('0') << setw(2);
		for (int j = 0; j < msg.size(); ++j) {
			cout <<  +(msg[j] & 0xff) << ' ';
		}
		cout << endl;
		
		// prepare rs232 packet
		char buffer[30];
		for (int j = 0; j < msg.size(); ++j) {
			buffer[j] = msg[j];
		}
		
		// send
		port.write(buffer, msg.size());
		
		// read
		bool res = port.read(buffer, 1, 200, 2);
		cout << "Received: " << res << buffer[0] << endl;
	}
	
	port.close();
	
	return 0;
}
