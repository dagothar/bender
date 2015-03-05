#ifndef _MCSPROTOCOL_DATACONVERSION_HPP
#define _MCSPROTOCOL_DATACONVERSION_HPP

#include "types.hpp"

namespace mcsprotocol {
	
/**
 * Provides methods for converting values to byte vectors expected by Schunk module.
 */
class DataConversion {
public:
	//! Converts float to byte vector.
	//static
	
	//! Converts byte vector to unsigned int.
	static unsigned int ByteVector2UnsignedInt(const ByteVector& bytes);
};

}

#endif // _MCSPROTOCOL_DATACONVERSION_HPP
