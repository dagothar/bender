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
	Response();
	virtual ~Response();
	
protected:

private:
};

}
}

#endif // _MCSPROTOCOL_RESPONSE_HPP
