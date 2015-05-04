cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(ar_object_listener src/ar_object_listener.cpp)
target_link_libraries(ar_object_listener json_prolog)
rosbuild_add_executable(ar_action_listener src/ar_action_listener.cpp)
target_link_libraries(ar_action_listener json_prolog)
rosbuild_add_executable(mytest src/mytest.cpp)
target_link_libraries(mytest json_prolog)
rosbuild_add_executable(export src/export.cpp)
target_link_libraries(export json_prolog)

rosbuild_add_executable(query_content_server src/query_content_server.cpp)
target_link_libraries(query_content_server json_prolog)
rosbuild_add_executable(query_content_client src/query_content_client.cpp)

rosbuild_add_boost_directories()
rosbuild_add_executable(get_drink_color_server src/get_drink_color_server.cpp)
rosbuild_link_boost(get_drink_color_server thread)

rosbuild_add_executable(color_trackbar src/color_trackbar.cpp)

rosbuild_include(rosprolog utils)

rosbuild_find_ros_package(ias_knowledge_base)
set( OWL_PATH_PREFIX ${ias_knowledge_base_PACKAGE_PATH} )
fix_ros_destination_path(OWL_PATH_PREFIX)

rosbuild_find_ros_package(infer_hidden_params)
set( LOCAL_PACKAGE_PATH ${infer_hidden_params_PACKAGE_PATH} )
fix_ros_destination_path(LOCAL_PACKAGE_PATH)

file(GLOB inputs owl/*.in)
foreach( _in ${inputs} )
string(REPLACE ".in" "" _out ${_in})
configure_file( ${_in} ${_out} )
endforeach( _in )
