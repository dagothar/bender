#ifndef _CRC_HPP
#define _CRC_HPP

#include <stdint.h>
#include <cstddef>
#include <vector>

/**
 * Interface for classes calculating CRC
 */
class CRC {
public:
	//! Destructor.
	virtual ~CRC() {}

	/**
	 * @brief Calculates CRC checksum of given data.
	 */
	virtual uint16_t crc(const std::vector<unsigned char> data) = 0;
};

#endif // _CRC_HPP
