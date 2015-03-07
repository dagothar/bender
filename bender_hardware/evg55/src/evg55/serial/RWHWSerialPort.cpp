#include "RWHWSerialPort.hpp"

using namespace rwhw;
using namespace rw::common;
using namespace evg55::serial;



RWHWSerialPort::RWHWSerialPort() {
	_port = ownedPtr(new rwhw::SerialPort());
}

RWHWSerialPort::~RWHWSerialPort() {
}

bool RWHWSerialPort::open(const std::string& name, unsigned baudrate) {
	switch (baudrate) { // TODO: other baudrates
		case 9600:
		default:
			return _port->open(name, (rwhw::SerialPort::Baud9600));
			break;
	};
	
	return false;
}

void RWHWSerialPort::close() {
	_port->close();
}

bool RWHWSerialPort::write(const char* buf, unsigned n) {
	return _port->write(buf, n);
}

int RWHWSerialPort::read(char* buf, unsigned n) {
	return _port->read(buf, n);
}

bool RWHWSerialPort::read(char* buf, unsigned n, unsigned timeout) {
	return _port->read(buf, n, timeout, 2);
}

void RWHWSerialPort::clean() {
	return _port->clean();
}
