cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

add_definitions(-DUSE_SDL -O2 -DLINUX -DVERSION=\\"0.2.6-5\\" -I -I/usr/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT )

include_directories(include)

set(UVC_SOURCES 
  src/luvcview/avilib.c
  src/luvcview/color.c
  src/luvcview/gui.c
  src/luvcview/utils.c
  src/luvcview/v4l2uvc.c)

add_library(uvc SHARED ${UVC_SOURCES})
target_link_libraries(uvc m SDL v4l2)


add_executable(luvcview src/main_luvcview.c)
target_link_libraries(luvcview  uvc)

rosbuild_add_executable(opencv_view src/main_opencv_view.cpp)
target_link_libraries(opencv_view   m v4l2 uvc )


rosbuild_add_executable(v4r_cam src/v4l_cam/v4l_cam_node.cpp src/v4l_cam/v4l_cam.cpp src/v4l_cam/generate_dynamic_reconfigure.cpp)
target_link_libraries(v4r_cam m v4l2 uvc)
rosbuild_link_boost(v4r_cam thread signals system)