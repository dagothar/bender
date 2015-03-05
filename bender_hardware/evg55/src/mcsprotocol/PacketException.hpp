#ifndef _MCSPROTOCOL_PACKETEXCEPTION_HPP
#define _MCSPROTOCOL_PACKETEXCEPTION_HPP

#include <stdexcept>

namespace evg55 {
namespace mcsprotocol {

/**
 * Base class for exceptions thrown by packet classes.
 */
class PacketException: public std::runtime_error {
public:
	PacketException() : std::runtime_error("") {}
	
	virtual const char* what() const throw() {
		return "Unknown packet error";
	}
};

/**
 * Exception for when accesing packet data out of bounds.
 */
class PacketOutOfBounds: public PacketException {
public:
	virtual const char* what() const throw() {
		return "Packet index out of bounds";
	}
};

/**
 * Exception thrown when packet is too short for expected data.
 */
class PacketTooShort: public PacketException {
public:
	virtual const char* what() const throw() {
		return "Packet too short";
	}
};

}
}

#endif // _MCSPROTOCOL_PACKETEXCEPTION_HPP
