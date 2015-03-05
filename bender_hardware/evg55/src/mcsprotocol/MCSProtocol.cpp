#include "MCSProtocol.hpp"
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <crc/SchunkCRC16.hpp>

using namespace std;
using namespace crc;

void MCSProtocol::printPacket(const MCSProtocol::Packet& msg) {
	ios init(NULL);
    init.copyfmt(clog);
    
	for (unsigned i = 0; i < msg.size(); ++i) {
		clog << hex << setfill('0') << setw(2) << +msg[i] << ' ';
	}
	clog << endl;
	
	clog.copyfmt(init);
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

	// append CRC
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
	
	//clog << "Sending: ";
	//printPacket(packet);
	
	return port->write(buf, packet.size());
}

bool MCSProtocol::receive(SerialPort* port, Message& message) {
	message.messageType = MessageNone;
	message.moduleId = 0;
	message.data.clear();
	
	// create and clean a buffer for incoming data
	const int BUF_LEN = 30;
	char buf[BUF_LEN];
	for (int i = 0; i < BUF_LEN; buf[i++] = 0);
	
	// read incoming message
	bool received = port->read(buf, BUF_LEN, 200);
	port->clean();
	
	// check if properly formatted message
	if (buf[0] != '\x07') return false;
	
	// calculate message length using dlen
	int msgLength = 3 + static_cast<unsigned char>(buf[2]);
	
	for (unsigned i = 0; i < msgLength; ++i) {
		message.data.push_back(buf[i]);
	}
	
	//clog << "Received: ";
	//printPacket(message.data);
	
	// get module id
	message.moduleId = message.data[1];
	
	// check message type
	if (message.data.size() < IndexCommand) return false;
	
	if (message.data[IndexData] == CommandOk1 && message.data[IndexData+1] == CommandOk2) {
		message.messageType = MessageOk;
	}
	else if (message.data[IndexCommand] == CommandError) {
		message.messageType = MessageError;
	}
	else if (message.data[IndexCommand] == CommandFailed) {
		message.messageType = MessageFailed;
	}
	else if (message.data[IndexCommand] == CommandMoveBlocked) {
		message.messageType = MessageMoveBlocked;
	}
	else if (message.data[IndexCommand] == CommandPositionReached) {
		message.messageType = MessagePositionReached;
	}
	else {
		message.messageType = MessageOther;
	}
	
	return true;
}

bool MCSProtocol::ping(SerialPort* port, unsigned char id) {
	MCSProtocol::Packet packet = makeGetStateCommand(id);
	
	if (!send(port, packet)) return false;
	
	Message response;
	if (!receive(port, response)) return false;
	
	if (response.messageType == MessageOther
		&& response.data[MCSProtocol::IndexCommand] == MCSProtocol::CommandGetState
		&& response.data[MCSProtocol::IndexModuleId] == id) {
		
		return true;
	}
	
	return false;
}

MCSProtocol::Data MCSProtocol::makeData(float value) {
	Data data;
	
	char* valueBytes = reinterpret_cast<char*>(&value);
	data.push_back(valueBytes[0]);
	data.push_back(valueBytes[1]);
	data.push_back(valueBytes[2]);
	data.push_back(valueBytes[3]);
	
	return data;
}

MCSProtocol::Packet MCSProtocol::makeReferenceCommand(unsigned char id) {
	Data data;
	data.push_back(CommandReference);
	
	return makePacket(id, data);
}

MCSProtocol::Packet MCSProtocol::makeGetStateCommand(unsigned char id, float interval) {
	Data data;
	data.push_back(CommandGetState);
	
	Data param = makeData(interval);
	data.insert(data.end(), param.begin(), param.end());
	
	return makePacket(id, data);
}

MCSProtocol::Packet MCSProtocol::makeMovePositionCommand(unsigned char id, float pos) {
	Data data;
	data.push_back(CommandMovePosition);
	
	Data position = makeData(pos);
	data.insert(data.end(), position.begin(), position.end());
	
	return makePacket(id, data);
}

MCSProtocol::Packet MCSProtocol::makeMoveCurrentCommand(unsigned char id, float cur) {
	Data data;
	data.push_back(CommandMoveCurrent);
	
	Data position = makeData(cur);
	data.insert(data.end(), position.begin(), position.end());
	
	return makePacket(id, data);
}

MCSProtocol::Packet MCSProtocol::makeMoveVelocityCommand(unsigned char id, float vel) {
	Data data;
	data.push_back(CommandMoveVelocity);
	
	Data param = makeData(vel);
	data.insert(data.end(), param.begin(), param.end());
	
	return makePacket(id, data);
}

MCSProtocol::Packet MCSProtocol::makeMoveGripCommand(unsigned char id, float cur) {
	Data data;
	data.push_back(CommandMoveGrip);
	
	Data param = makeData(cur);
	data.insert(data.end(), param.begin(), param.end());
	
	return makePacket(id, data);
}

MCSProtocol::Packet MCSProtocol::makeClearErrorCommand(unsigned char id) {
	Data data;
	data.push_back(CommandAcknowledge);
	
	return makePacket(id, data);
}
