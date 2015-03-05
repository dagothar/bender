#include "DataConversion.hpp"

using namespace evg55::mcsprotocol;

ByteVector DataConversion::float2byteVector(float value) {
	ByteVector bytes;
	
	unsigned char* buf = reinterpret_cast<unsigned char*>(&value);
	bytes.push_back(buf[0]);
	bytes.push_back(buf[1]);
	bytes.push_back(buf[2]);
	bytes.push_back(buf[3]);
	
	return bytes;
}

float DataConversion::byteVector2float(const ByteVector& bytes) {
	unsigned char buf[4] = { bytes[0], bytes[1], bytes[2], bytes[3] };
	const float* result = reinterpret_cast<float*>(buf);
	return *result;
}

unsigned int DataConversion::byteVector2unsignedInt(const ByteVector& bytes) {
	unsigned char buf[2] = { bytes[0], bytes[1] };
	const unsigned int* result = reinterpret_cast<unsigned int*>(buf);
	return *result;
}
