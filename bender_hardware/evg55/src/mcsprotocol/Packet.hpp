#ifndef _MCSPROTOCOL_PACKET_HPP
#define _MCSPROTOCOL_PACKET_HPP

#include <iostream>

#include "types.hpp"
#include "PacketException.hpp"

namespace mcsprotocol {

/**
 * A packet type for MCS protocol.
 */
class Packet {
public:	
	//! Defines byte locations.
	enum Indices {
		HeaderIndex = 0,
		IdIndex = 1,
		DlenIndex = 2,
		CmdIndex = 3,
		DataIndex = 4
	};
	
public:
	/* CONSTRUCTION */
	/**
	 * @brief Constructor.
	 * Construct packet from individual parts.
	 */
	Packet(Byte header = 0x00, Byte id = 0x00, Byte cmd = 0x00, const ByteVector& data = ByteVector());
	
	/**
	 * @brief Constructor.
	 * Constructs packet from raw data.
	 */
	Packet(const ByteVector& rawData);
	
	//! Destructor.
	virtual ~Packet();
	
	/* GETTERS & SETTERS */
	//! Returns packet size, including CRC bytes.
	size_t size() const;
	
	Byte& operator[](unsigned int i);
	const Byte& operator[](unsigned int i) const;
	
	Byte getHeader() const;
	void setHeader(Byte header);
	Byte getId() const;
	void setId(Byte id);
	Byte getCommand() const;
	void setCommand(Byte command);
	Byte getDlen() const;
	void setDlen(Byte dlen);
	ByteVector getData() const;
	void setData(const ByteVector& data);
	unsigned short getCrcSum() const;
	
	/* FRIENDS */
	//! Stream output operator.
	friend std::ostream& operator<<(std::ostream& stream, const Packet& packet);

protected:
	//! Returns the data length (pure data, excluding command byte)
	inline unsigned getDataLength() const;
	
	//! Returns the index of the beginning of the CRC sum.
	inline unsigned getCrcIndex() const;
	
	//! Updates the CRC sum.
	unsigned short updateCrc();

private:
	ByteVector _packet;
};

std::ostream& operator<<(std::ostream& stream, const Packet& packet);

}

#endif // _MCSPROTOCOL_PACKET_HPP
