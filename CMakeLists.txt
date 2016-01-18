cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#rosbuild_add_executable(isolate src/isolate.cpp)
#rosbuild_add_executable(track src/track.cpp)
#rosbuild_add_executable(filter src/filter.cpp)
#rosbuild_add_executable(geometry src/geometry.cpp)
rosbuild_add_executable(controller src/pilot_new.cpp)
#rosbuild_add_executable(controller2 src/pilot_face.cpp)
#rosbuild_add_executable(ar_companion src/ar_pose_companion.cpp)
rosbuild_add_executable(pose_filter src/filter2.cpp)
#rosbuild_add_executable(viso_companion src/viso2_companion.cpp)
rosbuild_add_executable(depth_handler src/depth_stuff.cpp)

#rosbuild_add_executable(path_vis src/path_vis.cpp)

#rosbuild_add_executable(isolate src/drec.cpp)
rosbuild_add_executable(las_nav src/laser_nav.cpp)
rosbuild_link_boost(controller thread)

rosbuild_add_executable(nav_talker src/nav_talker.cpp)
rosbuild_add_executable(nav_talker_rviz src/nav_talker_rviz.cpp)

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
