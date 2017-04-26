cmake_minimum_required(VERSION 2.8.3)
project(rviz_plugin_osvr)

#catkin
find_package(catkin REQUIRED COMPONENTS rviz)
catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Qt
set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
	message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSTION}")
	find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
else()
	message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSTION}")
   find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
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

add_library(${PROJECT_NAME} src/rviz_plugin_osvr.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES} ${OSVR_LIBRARIES})


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
