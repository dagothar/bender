#include "DataConversion.hpp"

using namespace evg55::mcsprotocol;

unsigned int DataConversion::ByteVector2UnsignedInt(const ByteVector& bytes) {
	unsigned char buf[2] = { bytes[0], bytes[1] };
	const unsigned int* result = reinterpret_cast<unsigned int*>(buf);
	return *result;
}
