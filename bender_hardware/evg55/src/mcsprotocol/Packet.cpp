#include "Packet.hpp"

#include <iomanip>

#include "DataConversion.hpp"

using namespace std;
using namespace mcsprotocol;



Packet::Packet(Byte header, Byte id, Byte cmd, const ByteVector& data) :
	_packet(6) // basic packet is of length 6: hdr + id + dlen + cmd + 2*crc
{
	setHeader(header);
	setId(id);
	setCommand(cmd);
	setData(data);
	
	updateCrc();
}

Packet::Packet(const ByteVector& rawData) :
	_packet(rawData)
{
}

Packet::~Packet() {
}

size_t Packet::size() const {
	return _packet.size();
}

Byte& Packet::operator[](unsigned int i) {
	if (i >= _packet.size()) throw PacketOutOfBounds();
	
	return _packet[i];
}

const Byte& Packet::operator[](unsigned int i) const {
	if (i >= _packet.size()) throw PacketOutOfBounds();
	
	return _packet[i];
}

Byte Packet::getHeader() const {
	return (*this)[HeaderIndex];
}

void Packet::setHeader(Byte header) {
	(*this)[HeaderIndex] = header;
	updateCrc();
}

Byte Packet::getId() const {
	return (*this)[IdIndex];
}

void Packet::setId(Byte id) {
	(*this)[IdIndex] = id;
	updateCrc();
}

Byte Packet::getCommand() const {
	return (*this)[CmdIndex];
}

void Packet::setCommand(Byte command) {
	(*this)[CmdIndex] = command;
	updateCrc();
}

Byte Packet::getDlen() const {
	return (*this)[DlenIndex];
}

ByteVector Packet::getData() const {	
	if (DataIndex + getDataLength() > _packet.size() - 2) throw PacketTooShort();
	
	return ByteVector(_packet.begin() + DataIndex, _packet.begin() + DataIndex + getDataLength());
}

void Packet::setData(const ByteVector& data) {
	// clear previous data
	if (DataIndex + getDataLength() > _packet.size() - 2) throw PacketTooShort();
	
	_packet.erase(_packet.begin() + DataIndex, _packet.begin() + DataIndex + getDataLength());
	
	// insert new data
	_packet.insert(_packet.begin() + DataIndex, data.begin(), data.end());
	
	// update Dlen
	(*this)[DlenIndex] = data.size() + 1;
	
	updateCrc();
}

unsigned short Packet::getCrcSum() const {
	if (getCrcIndex() + 2 > _packet.size()) throw PacketOutOfBounds();
	
	return DataConversion::ByteVector2UnsignedInt(ByteVector(_packet.begin() + getCrcIndex(), _packet.begin() + getCrcIndex() + 2));
}

unsigned Packet::getDataLength() const {
	return getDlen() - 1;
}

unsigned Packet::getCrcIndex() const {
	return CmdIndex + getDlen();
}

unsigned short Packet::updateCrc() const {
}

ostream& operator<<(ostream& stream, const Packet& packet) {
	// save stream state
	ios init(NULL);
    init.copyfmt(stream);
    
    const vector<Byte> bytes;// = packet.raw();
	for (unsigned i = 0; i < bytes.size(); ++i) {
		stream << hex << setfill('0') << setw(2) << +bytes[i] << ' ';
	}
	stream << endl;
	
	// restore stream state
	stream.copyfmt(init);
	return stream;
}
