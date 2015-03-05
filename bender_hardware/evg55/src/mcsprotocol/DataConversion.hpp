#ifndef _MCSPROTOCOL_DATACONVERSION_HPP
#define _MCSPROTOCOL_DATACONVERSION_HPP

#include "types.hpp"

namespace evg55 {
namespace mcsprotocol {
	
/**
 * Provides methods for converting values to byte vectors expected by Schunk module.
 */
class DataConversion {
public:
	//! Converts float to byte vector.
	static ByteVector float2byteVector(float value);
	
	//! Converts byte vector to float.
	static float byteVector2float(const ByteVector& bytes);
	
	//! Converts byte vector to unsigned int.
	static unsigned int byteVector2unsignedInt(const ByteVector& bytes);
};

}
}

#endif // _MCSPROTOCOL_DATACONVERSION_HPP
