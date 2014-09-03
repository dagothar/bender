CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

#-----------------------------------------------------------------------
# Set the name of the project

project(TrakStarTest)

#-----------------------------------------------------------------------
# Set paths to RobWork components
# Environment variables RW_* should be set up in .bashrc

set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(RW_ROOT "$ENV{RW_ROOT}")
set(RobWork_DIR ${RW_ROOT}/cmake)

# We use the settings that robwork studio uses
set(CMAKE_BUILD_TYPE Release)

# Include default settings for constructing a robwork dependent project
find_package(RobWork REQUIRED)
include_directories(${ROBWORK_INCLUDE_DIRS})
link_directories(${ROOT} ${ROBWORK_LIBRARY_DIRS})

# Set the output dir for generated libraries and binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")

#-----------------------------------------------------------------------
# Build targets

add_executable(trakstar-test main.cpp Trakstar.cpp)

#-----------------------------------------------------------------------
# Add libraries

target_link_libraries(trakstar-test ${CMAKE_CURRENT_SOURCE_DIR}/atc3dg.lib ${ROBWORK_LIBRARIES} )
