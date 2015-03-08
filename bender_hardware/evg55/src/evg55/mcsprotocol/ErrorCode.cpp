#include "ErrorCode.hpp"

using namespace std;
using namespace evg55::mcsprotocol;

string ErrorCode::errorToString(unsigned char errorCode) {
	
	switch (errorCode) {
		case 0xd5:
			return "ERROR SOFT LOW";
			
		case 0xd6:
			return "ERROR SOFT_HIGH";
			
		// TODO: add more errors
		
		default:
			return "Unknown error";
	}
	
	// should not end here
	return "";
}
