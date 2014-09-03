CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project.
PROJECT(SkywalkerPlugin)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(RW_ROOT "$ENV{RW_ROOT}")
SET(RWS_ROOT "$ENV{RWS_ROOT}")

SET(RobWork_DIR ${RW_ROOT}/cmake)
SET(RobWorkStudio_DIR ${RWS_ROOT}/cmake)

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)

#Include default settings for constructing a robwork dependent project
FIND_PACKAGE(RobWork REQUIRED)
FIND_PACKAGE(RobWorkStudio REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS})
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS})

# Set the output dir for generated libraries and binaries
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${ROOT}/bin" CACHE PATH "Runtime directory" FORCE )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Library directory" FORCE )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${ROOT}/libs" CACHE PATH "Archive directory" FORCE )

#########################
#From here we add the plugins
SET(UisFiles skywalker.ui)

SET(MocSrcFiles )
QT4_WRAP_CPP(MocSrcFiles SkywalkerPlugin.hpp)
SET(RccSrcFiles )
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)
QT4_WRAP_UI(UiHeaders ${UisFiles} )

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(doc
	${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")

#MESSAGE(${ROBWORKSIM_LIBRARIES})
#MESSAGE("!" ${ODE_LIBRARIES})
# The shared library to build
add_library( SkywalkerPlugin MODULE SkywalkerPlugin.cpp FPPController.cpp ${MocSrcFiles} ${RccSrcFiles} ${UiHeaders})

target_link_libraries( SkywalkerPlugin  ${ROBWORK_LIBRARIES} ${ROBWORKSTUDIO_LIBRARIES})

