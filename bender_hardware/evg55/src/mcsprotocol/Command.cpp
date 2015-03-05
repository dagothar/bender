#include "Command.hpp"

#include "DataConversion.hpp"

using namespace evg55::mcsprotocol;

Command::Command(Byte id, Byte cmd) :
	Packet(0x05, id, cmd) {
}

Command::~Command() {
}

void Command::addParameter(float value) {
	ByteVector data = getData();
	
	ByteVector newParam = DataConversion::float2byteVector(value);
	data.insert(data.end(), newParam.begin(), newParam.end());
	
	setData(data);
}
