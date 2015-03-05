#ifndef _MCSPROTOCOL_RESPONSE_HPP
#define _MCSPROTOCOL_RESPONSE_HPP

#include "Packet.hpp"

namespace evg55 {
namespace mcsprotocol {
	
/**
 * Represent a response received from MCS module.
 */
class Response: public Packet {
public:
	//! Constructor.
	Response();
	
	/**
	 * @brief Constructor.
	 */
	Response(const ByteVector& rawData);
	
	//! Destructor.
	virtual ~Response();
	
	/**
	 * @brief Check response sanity.
	 * Response is ok, when:
	 * - crc sum matches,
	 * - the header is 0x07,
	 * - response is of proper length
	 * 
	 * @return Returns \b true when response is formatted properly.
	 */
	bool isOk() const;
	
	//! Returns \b true if it's an acknowledgement ("OK") message.
	bool isAcknowledgement() const;
	
	//! Returns \b true if it's an error message.
	bool isError() const;
	
	//! Returns \b true if it's a GetState response.
	bool isState() const;
};

}
}

#endif // _MCSPROTOCOL_RESPONSE_HPP
