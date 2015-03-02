#ifndef _SCHUNKCRC16_HPP
#define _SCHUNKCRC16_HPP

#include "CRC.hpp"



class SchunkCRC16: public CRC {
public:
	//! @copydoc CRC:crc
	virtual uint16_t crc(const std::vector<unsigned char> data);

protected:
	/**
	 * @brief Implements Schunk CRC16 routine as found in Motion Control pdf
	 */
	virtual uint16_t doCrc(uint16_t prevCrc, unsigned char data);
};

#endif // _SCHUNKCRC16_HPP
