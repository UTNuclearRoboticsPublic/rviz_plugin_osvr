cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin_osvr)

#catkin
find_package(catkin REQUIRED COMPONENTS rviz)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

#Boost
#find_package(Boost REQUIRED)
#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARIES})

# Qt
#set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
	message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
	#find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
	#include(${QT_USE_FILE})
else()
	message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
   find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
	include_directories(${Qt5Widgets_INCLUDE_DIRS})
	message(STATUS "QT widgets includes: ${Qt5Widgets_INCLUDE_DIRS} ")
	#include_directories(${Qt5Gui_INCLUDE_DIRS})
	#message(STATUS "QT Gui includes: ${Qt5Gui_INCLUDE_DIRS} ")
	#include_directories(${Qt5Core_INCLUDE_DIRS})
	#message(STATUS "QT Core includes: ${Qt5Core_INCLUDE_DIRS} ")
endif()


add_definitions(-DQT_NO_KEYWORDS)

# OSVR
set(OSVR_DIR ~/osvr)
find_package(osvr REQUIRED PATHS ${OSVR_DIR})
if(osvr_FOUND)
	message(STATUS "Found osvr version ${osvr_VERSION} in ${osvr_DIR}")
	set(OSVR_INCLUDE_DIRS ${OSVR_DIR}/include)
	set(OSVR_LIBRARIES ${OSVR_DIR}/lib/libosvrClientKit.so ) 

	message(STATUS "OSVR INCLUDE DIRS: ${OSVR_INCLUDE_DIRS}")
	message(STATUS "OSVR LIBRARIES: ${OSVR_LIBRARIES}")
	
	include_directories(${OSVR_INCLUDE_DIRS})
else()
	message(FATAL_ERROR "osvr")
endif()

find_package(PkgConfig REQUIRED) # required for packaged below

# OGRE
pkg_check_modules(OGRE OGRE)
include_directories(${OGRE_INCLUDE_DIRS})
message(STATUS "OGRE LIBRARIES: ${OGRE_INCLUDE_DIRS}")

# JSONCPP
pkg_check_modules(JSONCPP jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})
message(STATUS "jsoncpp LIBRARIES: ${JSONCPP_INCLUDE_DIRS}")



set(SRC_FILES
	src/ogre_osvr.cpp
	src/plugin_display.cpp
	src/distortion.cpp
	)

#generate Qt MOC files
qt5_wrap_cpp(MOC_FILES
	include/rviz_plugin_osvr/plugin_display.h
	include/rviz_plugin_osvr/ogre_osvr.h
	include/rviz_plugin_osvr/distortion.h
	)

add_library(${PROJECT_NAME} ${SRC_FILES} ${MOC_FILES})

SET(CMAKE_SHARED_LINKER_FLAGS -Wl,--no-allow-shlib-undefined,--no-undefined)
#SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES
#	PREFIX ""
#	LINK_FLAGS "-Wl, --no-allow-shlib-undefined"
#	)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OSVR_LIBRARIES} ${JSONCPP_LIBRARIES})


#############
## Install ##
#############

#install(TARGETS rviz_plugin_osvr
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
#
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
#
#install(DIRECTORY rviz_media
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#
#install(FILES
#  plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#
#install(DIRECTORY
#  icons
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
