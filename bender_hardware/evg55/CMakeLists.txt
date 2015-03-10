# Test CMake version
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# The name of the project. (EDIT THIS)
PROJECT(evg55)

# optionally: point the find package in the direction of the robwork.
SET(RobWork_DIR $ENV{RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)

SET(RWHW_INCLUDES $ENV{RWHW_ROOT}/src)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

# if you have additional libraries or include dirs then add them here
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(USER_LIBRARIES )
INCLUDE_DIRECTORIES(${ROOT}/src/ ${RWHW_INCLUDES})
LINK_DIRECTORIES($ENV{RWHW_ROOT}/libs/Release)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE)
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE)

# set source files
SET(FILES_CPP CACHE INTERNAL "" FORCE)
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tools)

# And now we add any targets that we want
add_library(evg55 ${FILES_CPP})
target_link_libraries(evg55 ${USER_LIBRARIES} rwhw_serialport ${ROBWORK_LIBRARIES})
