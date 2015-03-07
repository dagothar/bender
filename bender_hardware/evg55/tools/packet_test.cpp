#include <iostream>
#include <evg55/mcsprotocol/Command.hpp>
#include <evg55/mcsprotocol/CommandFactory.hpp>

using namespace std;
using namespace evg55::mcsprotocol;



int main() {
	Packet packet(0x05, 0x12, 0x95);
	cout << packet << endl;
	
	packet.setHeader(0x07);
	cout << packet << endl;
	
	packet.setId(0x13);
	cout << packet << endl;
	
	ByteVector data;
	data.push_back(0x4f);
	data.push_back(0xfb);
	packet.setData(data);
	cout << packet << endl;
	
	cout << +packet.getHeader() << endl;
	
	ByteVector raw;
	raw.push_back(0x05);
	raw.push_back(0x12);
	raw.push_back(0x05);
	raw.push_back(0xb0);
	raw.push_back(0x00);
	raw.push_back(0x00);
	raw.push_back(0x20);
	raw.push_back(0x42);
	raw.push_back(0x42);
	raw.push_back(0x42);
	Packet packet1(raw);
	cout << packet1 << endl;
	cout << packet1.getCrcSum() << endl;
	
	packet1.updateCrc();
	cout << packet1 << endl;
	cout << packet1.getCrcSum() << endl;
	
	Command cmd = CommandFactory::makeGetStateCommand(0x12);
	cout << cmd << endl;
	
	Command cmd1 = CommandFactory::makeMovePositionCommand(0x12, 0.0);
	cout << cmd1 << endl;
	
	cout << cmd1.getCrcSum() << " == " << cmd1.calculateCrcSum() << endl;
	cout << cmd1.checkCrc() << endl;
	
	return 0;
}
