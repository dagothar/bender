#####################################################
# Template for building RobWork dependent console application
#
# You should edit directly in this file to add extra source 
# files and when configuring your project.  
#####################################################

# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. (EDIT THIS)
PROJECT(GripperTest)

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
add_executable(GripperTest GripperTest.cpp SchunkEVG55.cpp)
target_link_libraries(GripperTest  ${USER_LIBRARIES}   rwhw_pcube rwhw_serialport ${ROBWORK_LIBRARIES})

