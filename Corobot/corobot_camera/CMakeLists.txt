cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set(ROS_BUILD_TYPE Release)

rosbuild_init()

rosbuild_gensrv()
rosbuild_genmsg()

find_package(OpenCV)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

pkg_check_modules(SDL REQUIRED sdl)
include_directories(${SDL_INCLUDE_DIRS})
link_directories(${SDL_LIBRARY_DIRS})
add_definitions("-D_FILE_OFFSET_BITS=64")

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# add the uvc_cam library
rosbuild_add_library(uvc_cam src/uvc_cam/uvc_cam.cpp)
target_link_libraries(uvc_cam avcodec swscale)

rosbuild_add_executable(view src/view/view.cpp)
target_link_libraries(view uvc_cam)
target_link_libraries(view ${SDL_LIBRARIES})

rosbuild_add_executable(enumerate src/enumerate/enumerate.cpp)
target_link_libraries(enumerate uvc_cam)

rosbuild_add_executable(dump src/dump/dump.cpp src/dump/avilib.c)
target_link_libraries(dump uvc_cam)

#rosbuild_add_executable(play src/play.cpp)
#target_link_libraries(play uvc_cam)

rosbuild_add_executable(dynamic_sender src/dynamic_sender.cpp)
target_link_libraries(dynamic_sender uvc_cam)
target_link_libraries(dynamic_sender ${OpenCV_LIBRARIES})

rosbuild_add_executable(record_theora src/record_theora.cpp)
target_link_libraries(record_theora theora theoraenc theoradec uvc_cam)

rosbuild_add_executable(dump_send src/dump/dump_send.cpp src/dump/avilib.c)
target_link_libraries(dump_send uvc_cam)
