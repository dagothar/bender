#ifndef _EVG55_MCSPROTOCOL_ERRORCODE_HPP
#define _EVG55_MCSPROTOCOL_ERRORCODE_HPP

#include <string>
#include <map>

namespace evg55 {
namespace mcsprotocol {
	
/**
 * Decode cryptic Schunk error codes.
 */
class ErrorCode {
public:
	static std::string errorToString(unsigned char errorCode);
};

}
}

#endif // _EVG55_MCSPROTOCOL_ERRORCODE_HPP
