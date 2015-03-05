#include "Response.hpp"

#include "Command.hpp"

using namespace evg55::mcsprotocol;

Response::Response() {
}

Response::Response(const ByteVector& rawData) :
	Packet(rawData) {
}

Response::~Response() {
}

bool Response::isOk() const {
	//TODO: include other conditions?
	
	return checkCrc() && getHeader() == 0x07 && size() == getDlen() + 5;
}

bool Response::isAcknowledgement() const {
	//TODO: make sure it's ok
	
	if (getDlen() < 3) return false;
	
	ByteVector data = getData();
	if (data[0] == 'O' && data[1] == 'K') return true;
	
	return false;
}

bool Response::isError() const {
	//TODO: check if this captures any possible error message
	
	return getCommand() == Command::Error;
}

bool Response::isState() const {
	return getCommand() == Command::GetState && getDlen() == 0x0f;
}
