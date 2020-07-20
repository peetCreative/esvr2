cmake_minimum_required( VERSION 3.9 )
project( esvr2 )

include( CMake/Bootstrap.cmake )
include( CMake/Dependencies/OGRE.cmake )

include( CMake/FindPkgMacros.cmake )
find_package( OpenCV REQUIRED )
message("OpenCV Libs: ${OpenCV_LIBS}")
message("OpenCV Include : ${OpenCV_INCLUDE_DIRS}")
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-class-memaccess" )
option( BUILD_ROS "compile ROS Nodelet" ON )
#set( BUILD_ROS catkin_FOUND )
if (BUILD_ROS)
    find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        roscpp
        sensor_msgs
        geometry_msgs
        message_filters
        pluginlib)
    MESSAGE("build ROS")
    catkin_package()
    set( CMAKE_CURRENT_BIN_DIR "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}" )
    message( "CMAKE_CURRENT_BIN_DIR is ${CMAKE_CURRENT_BIN_DIR}" )
    add_definitions(-DPLUGIN_FOLDER="${CMAKE_CURRENT_BIN_DIR}/${CMAKE_BUILD_TYPE}/" )
    add_definitions(-DRESOURCE_FOLDER="${CMAKE_CURRENT_BIN_DIR}/Data/")
    add_definitions(-DUSE_ROS)
#     generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)

#     catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)
else()
    set( CMAKE_CURRENT_BIN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/bin" )
    set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin/${CMAKE_BUILD_TYPE}" )
endif()

set(CMAKE_CXX_FLAGS "-Wno-class-memaccess")
setupOgre( OGRE_SOURCE, OGRE_BINARIES, OGRE_LIBRARIES, FALSE )

find_package(PkgConfig)

pkg_check_modules(YAML_CPP yaml-cpp)
if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif()
link_directories(${YAML_CPP_LIBRARY_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

# set(OpenVR_DIR /usr/local/include/openvr/)
#TODO: write proper findlibconfig++.cmake to find this library
# find_package(libconfig++ REQUIRED)
#TODO: automatically find package make the
# find_package( OpenVR REQUIRED )
if (NOT OpenVR_FOUND)
	message("no openvr")
	set(OpenVR_INCLUDE_DIRS /usr/local/include/openvr/)
	set(OpenVR_LIBRARIES /usr/local/lib/libopenvr_api.so)
endif()

# Setup our application
if( MSVC )
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
	if( NOT PLATFORM_X64 )
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:SSE2")
	endif()
	add_definitions( -DUNICODE -D_UNICODE )
endif()

macro( add_recursive dir retVal )
    file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

include_directories( "${CMAKE_CURRENT_SOURCE_DIR}/include" ${OpenVR_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} )

link_libraries(stdc++fs)

add_recursive( ${CMAKE_CURRENT_SOURCE_DIR}/src SOURCES )
add_recursive( ${CMAKE_CURRENT_SOURCE_DIR}/include HEADERS )

if (BUILD_ROS)
    include_directories("${catkin_INCLUDE_DIRS}")
    add_executable( ${PROJECT_NAME}Node
        ${SOURCES} ${SOURCES_ROS} ${HEADERS} ${RESOURCES} )
    target_link_libraries( ${PROJECT_NAME}Node
        ${catkin_LIBRARIES} ${OGRE_LIBRARIES}
        ${OpenVR_LIBRARIES} ${OpenCV_LIBS} config++ ${YAML_CPP_LIBRARIES})
else()
    add_executable( ${PROJECT_NAME} ${SOURCES} ${HEADERS} ${RESOURCES} )
    target_link_libraries( ${PROJECT_NAME} ${OGRE_LIBRARIES} ${OpenVR_LIBRARIES} ${OpenCV_LIBS} config++ ${YAML_CPP_LIBRARIES})
endif()
