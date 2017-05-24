# rviz\_plugin\_osvr
## Introduction
Creates the [OSVR](http://www.osvr.com) stereo display for RViz. It wraps the IMU orientation and the headset position (tracked from IR camera) to control the view of the display. The rendered 3D context is provided by RVIz.


*This plugin is inspired by [oculus\_rvis\_plugins](https://github.com/ros-visualization/oculus_rviz_plugins) package.* 

## Build Instructions
### rviz\_plugin\_osvr package
Clone the package into your catkin workspace, build as any other ROS package.
```
git clone https://github.com/Veix123/rviz-plugin-osvr.git
catkin build
```

### OSVR library
First, let's make sure we have the required [dependencies](https://github.com/OSVR/OSVR-Docs/blob/master/Getting-Started/Installing/Linux-Build-Instructions.md):

```
sudo apt update && sudo apt upgrade
sudo apt install libsdl2-dev libopencv-dev libboost1.58-dev libboost-thread1.58-dev libboost-program-options1.58-dev libboost-filesystem-dev libusb-1.0-0-dev
```

Make a separate folder for build files and clone [OSVR-Core](https://github.com/OSVR/OSVR-Core) and [libfunctionality](https://github.com/OSVR/libfunctionality) repos. Let's install everything under osvr in our home directory starting with libfunctionality:
```
mkdir ~/build && cd ~/build
git clone --recursive https://github.com/OSVR/libfunctionality.git
git clone --recursive https://github.com/OSVR/OSVR-Core.git

#Build libfunctionality
cd libfunctionality
cmake . -DCMAKE_INSTALL_PREFIX=~/osvr
make
make install
```

The OpenCV3 in ROS-kinetic comes with .cmake files, which are responsible setting up include directories, library paths and all other variables for the build environment. In cmake, projects can be set-up using configurations (such as Release, Debug, etc.). OpenCV defines only one configuration called `NULL`, while osvr uses cmake's default configuration called `RelWithDebInfo`. Hence, running cmake with the default build type will cause opencv to not find its libraries and fails with `*** No rule to make target 'opencv_core-NOTFOUND' ***`.
The solution is remapping RelWithDebInfo configuration to NONE for each opencv module. I also had to manually bring in the opencv include directories, since some test modules were unable to find OpenCV headers.
So, locate the following section in OSVR-Core/CMakeLists.txt:
```
if(OPENCV_MODULES_USED)
    list(REMOVE_DUPLICATES OPENCV_MODULES_USED)
endif()
```
and replace with:
```
if(OPENCV_MODULES_USED)
    list(REMOVE_DUPLICATES OPENCV_MODULES_USED)
    # Set configuration mapping from RelWithDebInfo to NONE for each imported opencv module
    foreach(_cvMod ${OPENCV_MODULES_USED})
        message(STATUS "Adding mapping to component: ${_cvMod}")
        set_property(TARGET ${_cvMod} APPEND PROPERTY MAP_IMPORTED_CONFIG_RELWITHDEBINFO NONE)
    endforeach()
    include_directories("${OpenCV_INCLUDE_DIRS}")
endif()
```

Now we're ready to build osvr with OpenCV. Remember to build it out of source using custom install prefix. You can make compiling faster with "make -j4" (or any other integer) on a multi-core system.

```
#Build OSVR-core
mkdir ~/build/OSVR-Core/build
cd ~/build/OSVR-Core/build
cmake .. -DCMAKE_INSTALL_PREFIX=~/osvr
make
make install
```

Before running the server, make sure you set the following udev rules (e.g. in `/etc/udev/rules.d/50-OSVR.rules`) or you have to start the server in privileged mode (see [OSVR-Core issue #330](https://github.com/OSVR/OSVR-Core/issues/330) for more details)
```
SUBSYSTEM=="usb", ATTR{idVendor}=="1532", ATTR{idProduct}=="0b00", MODE="0666", GROUP="plugdev" # osvr sensors
SUBSYSTEM=="usb", ATTR{idVendor}=="0572", ATTR{idProduct}=="1806", MODE="0666", GROUP="plugdev" # osvr audio
SUBSYSTEM=="usb", ATTR{idVendor}=="0bda", ATTR{idProduct}=="57e8", MODE="0666", GROUP="plugdev" # tracker camera for uvc-camera
```
And we're good to roll.
Let's fire up the server with default config for HDK2 headset.  
`~/osvr/bin/osvr_server ~/osvr/share/osvrcore/osvr_server_config.json`

Open another terminal and verify that osvr is working.  
`~/osvr/bin/OpenGLSample`

## USAGE
The plugin needs a running `osvr_server`, which you can either add to a system start-up or run via launch scripts:  
`roslaunch rviz_plugin_osvr ubot_demo.launch`  
Open up RViz, load the osvr display plugin, and have fun!
