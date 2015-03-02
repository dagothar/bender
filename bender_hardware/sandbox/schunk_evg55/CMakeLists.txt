# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. (EDIT THIS)
PROJECT(crc_test)

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR /home/dagothar/robwork/trunk/RobWork/cmake)
FIND_PACKAGE(RobWork REQUIRED)

SET(RWHW_INCLUDES /home/dagothar/robwork/trunk/RobWorkHardware/src)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if you have additional libraries or include dirs then add them here
SET(USER_LIBRARIES )
INCLUDE_DIRECTORIES(${ROOT}/src/ ${RWHW_INCLUDES})
LINK_DIRECTORIES(/home/dagothar/robwork/trunk/RobWorkHardware/libs/Release)

# And now we add any targets that we want
add_executable(crc_test test.cpp SchunkCRC16.cpp)
target_link_libraries(crc_test ${USER_LIBRARIES} rwhw_serialport ${ROBWORK_LIBRARIES})

add_executable(gripper_test gripper_test.cpp SchunkCRC16.cpp SchunkEVG55Gripper.cpp RWHWSerialPort.cpp MCSProtocol.cpp)
target_link_libraries(gripper_test ${USER_LIBRARIES} rwhw_serialport ${ROBWORK_LIBRARIES})

add_executable(cmd_test cmd_test.cpp SchunkCRC16.cpp SchunkEVG55Gripper.cpp RWHWSerialPort.cpp MCSProtocol.cpp)
target_link_libraries(cmd_test ${USER_LIBRARIES} rwhw_serialport ${ROBWORK_LIBRARIES})
