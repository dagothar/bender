#include "MCSProtocol.hpp"
#include <iostream>
#include <iomanip>
#include "SchunkCRC16.hpp"

using namespace std;

void MCSProtocol::printPacket(const MCSProtocol::Packet& msg) {
	ios init(NULL);
    init.copyfmt(cout);
    
	for (unsigned i = 0; i < msg.size(); ++i) {
		cout << hex << setfill('0') << setw(2) << +msg[i] << ' ';
	}
	cout << endl;
	
	cout.copyfmt(init);
}

MCSProtocol::Packet MCSProtocol::makePacket(unsigned char id, Data data) {
	MCSProtocol::Packet msg;

	// add 0x05 header (M -> S)
	msg.push_back('\x05');

	// add module id
	msg.push_back(id);

	// add dlen
	msg.push_back(static_cast<unsigned char>(data.size()));

	// add data
	msg.insert(msg.end(), data.begin(), data.end());

	// calculate CRC
	CRC* crcCalculator = new SchunkCRC16();
	uint16_t crcSum = crcCalculator->crc(msg);
	delete crcCalculator;

	char* crcBytes = reinterpret_cast<char*>(&crcSum);
	msg.push_back(crcBytes[0]);
	msg.push_back(crcBytes[1]);
	
	return msg;
}

bool MCSProtocol::send(SerialPort* port, const Packet& packet) {
	char buf[packet.size()];
	for (unsigned i = 0; i < packet.size(); ++i) {
		buf[i] = packet[i];
	}
	
	return port->write(buf, packet.size());
}

bool MCSProtocol::receive(SerialPort* port, unsigned char id, Packet& packet) {
	const int BUF_LEN = 30;
	
	// read BUF_LEN characters from the serial port
	char buf[BUF_LEN];
	bool received = port->read(buf, BUF_LEN, 200);
	port->clean();
	
	// check if properly formatted message
	if (buf[0] != '\x07') return false;
	
	// calculate message length using dlen
	int msgLength = 3 + static_cast<unsigned char>(buf[2]);
	
	Packet response;
	for (unsigned i = 0; i < msgLength; ++i) {
		response.push_back(buf[i]);
	}
	packet = response;
	
	printPacket(response);
	
	return true;
}

bool MCSProtocol::ping(SerialPort* port, unsigned char id) {
	// send a GET command
	MCSProtocol::Data data = makeData(GetState);
	MCSProtocol::Packet packet = makePacket(id, data);
	
	if (!send(port, packet)) return false;
	
	Packet response;
	if (!receive(port, id, response)) return false;
	
	if (response[3] == GetState) {
		return true;
	}
	
	return false;
}

bool MCSProtocol::isOk(const Packet& packet) {
	// the packet is ok if it containts 0x4f 0x4b at rth and 5th byte
	
	if (packet[4] == '\x4f' && packet[5] == '\x4b') {
		return true;
	}
	
	return false;
}

bool MCSProtocol::homeCmd(SerialPort* port, unsigned char id) {
	// send a REF command
	MCSProtocol::Data data = makeData(Reference);
	MCSProtocol::Packet packet = makePacket(id, data);
	
	if (!send(port, packet)) return false;
	
	Packet response;
	receive(port, id, response);
	
	if (isOk(response)) return true;
	
	return false;
}

bool MCSProtocol::movePositionCmd(SerialPort* port, unsigned char id, float pos) {
	MCSProtocol::Data data = makeData(MovePosition, pos);
	MCSProtocol::Packet packet = makePacket(id, data);
	
	if (!send(port, packet)) return false;
}

MCSProtocol::Data MCSProtocol::makeData(Command cmd) {
	Data data;
	
	data.push_back(static_cast<unsigned char>(cmd));
	
	return data;
}

MCSProtocol::Data MCSProtocol::makeData(Command cmd, float value) {
	Data data;
	
	data.push_back(static_cast<unsigned char>(cmd));
	
	char* valueBytes = reinterpret_cast<char*>(&value);
	data.push_back(valueBytes[0]);
	data.push_back(valueBytes[1]);
	data.push_back(valueBytes[2]);
	data.push_back(valueBytes[3]);
	
	return data;
}
