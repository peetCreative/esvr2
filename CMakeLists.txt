cmake_minimum_required( VERSION 3.16 )
project( Esvr2 )

SET(CMAKE_MODULE_PATH
        "${Esvr2_SOURCE_DIR}/CMake"
        "${Esvr2_SOURCE_DIR}/CMake/Utils"
        "${Esvr2_SOURCE_DIR}/CMake/Packages")

set( CMAKE_BUILD_TYPE Debug CACHE STRING "One of: Debug Release RelWithDebInfo MinSizeRel." FORCE )

set( CMAKE_CXX_FLAGS_DEBUG			"${CMAKE_CXX_FLAGS_DEBUG}			-DDEBUG=1 -D_DEBUG=1 -g"	)
set( CMAKE_CXX_FLAGS_RELEASE		"${CMAKE_CXX_FLAGS_RELEASE}			-O2 -DNDEBUG"				)
set( CMAKE_CXX_FLAGS_RELWITHDEBINFO	"${CMAKE_CXX_FLAGS_RELWITHDEBINFO}	-O2 -g"						)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-overloaded-virtual" )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-class-memaccess" )

if( CMAKE_CXX_COMPILER_ID MATCHES "Clang" )
    set( CMAKE_CXX_FLAGS_DEBUG		"${CMAKE_CXX_FLAGS_DEBUG}			-fno-limit-debug-info"		)
endif()

# FROM: https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")

set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/bin" )
set( LIBRARY_OUTPUT_PATH  "${CMAKE_CURRENT_BINARY_DIR}/lib" )

add_subdirectory(pivot_control_messages)

macro(SUBDIRLIST result curdir)
    FILE(GLOB_RECURSE children LIST_DIRECTORIES true ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${child} AND NOT "${child}" STREQUAL "Deprecated")
            SET(dirlist "${dirlist};${child}")
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
endmacro()
macro( add_recursive_lib dir retVal suffix)
    file( GLOB_RECURSE ${retVal} ${dir}/*${suffix}.so )
endmacro()

option(VERBOSE_CMAKE_MESSAGES "show more CMake messages" false)
set(ESVR2_OGRE_HOME  "OGRE_NOT_FOUND" CACHE PATH "path to install folder of ogre-next")
option(ESVR2_LOWLATENCY_VIDEOLOADER "Compile the Low-Latency-Videoloader" False)
option(ESVR2_BLACKMAGIC_VIDEOLOADER "Compile the BlackMagic VideoLoader" True)
SET(ESVR2_BLACKMAGIC_HOME "BLACKMAGIC_NOT_FOUND" CACHE PATH "path to install folder of blackmagic")
option(ESVR2_ANALOG_FOOTPEDAL "Compile the Low-Latency-Videoloader" False)
option(ESVR2_POINTCLOUD "Compile Pointcloud overlays. (Not Working)" False)
option(ESVR2_DOUBLE_PRECISION "Compile with double precision. Depend Ogre OGRE_CONFIG_DOUBLE" False)

#SUBDIRLIST( OGRE_INCLUDE_DIRS ${OGRE_HOME}/include)
SET(OGRE_HOME ${ESVR2_OGRE_HOME})
MESSAGE("-- Looking for ogre-next in OGRE_HOME: ${OGRE_HOME}")
if(${OGRE_HOME} STREQUAL "OGRE_NOT_FOUND")
    MESSAGE(FATAL_ERROR "Ogre path not defined. Exit CMake!")
endif()
SET(OGRE_INCLUDE_DIR ${OGRE_HOME}/include/OGRE)
SET(OGRE_INCLUDE_DIRS "${OGRE_INCLUDE_DIR}"
        "${OGRE_INCLUDE_DIR}/Overlay"
        "${OGRE_INCLUDE_DIR}/RenderSystems/GL3Plus"
        "${OGRE_INCLUDE_DIR}/Hlms/Unlit"
        "${OGRE_INCLUDE_DIR}/Hlms/Common"
        )
if(${CMAKE_BUILD_TYPE} MATCHES Debug)
    if (VERBOSE_CMAKE_MESSAGES)
        message("Debug")
    endif()
    add_recursive_lib(${OGRE_HOME}/lib OGRE_LIBRARIES "_d")
else()
    if (VERBOSE_CMAKE_MESSAGES)
        message("Release")
    endif()
    add_recursive_lib(${OGRE_HOME}/lib OGRE_LIBRARIES "")
endif()
SET(OGRE_MEDIA_DIR ${OGRE_HOME}/share/OGRE/Media)
SET(OGRE_VERSION 2.2.4)

#SET(Ogre_FOUND TRUE)
#SET(Ogre_INCLUDE_DIRS /opt )
#  Ogre_LIBRARIES - link these to use the Ogre core
#  Ogre_BINARY_REL - location of the main Ogre binary (win32 non-static only, release)
#  Ogre_BINARY_DBG - location of the main Ogre binaries (win32 non-static only, debug)


#find_package( Ogre PATHS ${Ogre_DIR} NO_DEFAULT_PATH REQUIRED)
if(NOT ${OGRE_VERSION} VERSION_GREATER 2.2.0)
    message("OGRE_INCLUDE_DIRS:${OGRE_INCLUDE_DIRS}")
    message("OGRE_LIBRARY_REL:${OGRE_LIBRARY_REL}")
    message(SEND_ERROR "Too Low Ogre Version (${OGRE_VERSION}) found")
else()
    if (VERBOSE_CMAKE_MESSAGES)
        message("OGRE_INCLUDE_DIRS:${OGRE_INCLUDE_DIRS}")
        message("OGRE_LIBRARY_REL:${OGRE_LIBRARY_REL}")
    endif()
endif()

find_package(SDL2 REQUIRED)
if(ESVR2_ANALOG_FOOTPEDAL)
    find_package(SerialPort REQUIRED)
    add_definitions(-DUSE_FOOTPEDAL)
endif()

find_package( OpenCV 4.2 REQUIRED )
if (VERBOSE_CMAKE_MESSAGES)
    message("OpenCV Version: ${OpenCV_VERSION}")
    message("OpenCV Libs: ${OpenCV_LIBS}")
    message("OpenCV Include : ${OpenCV_INCLUDE_DIRS}")
endif()

#find_package(PkgConfig)

find_package( yaml-cpp REQUIRED )
if (VERBOSE_CMAKE_MESSAGES)
    Message(yaml-cpp_VERSION:${yaml-cpp_VERSION})
endif()
#pkg_check_modules(YAML_CPP yaml-cpp)
if(${yaml-cpp_VERSION} VERSION_GREATER 0.5)
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif()
link_directories(${yaml-cpp_LIBRARY_DIRS})
include_directories(${yaml-cpp_INCLUDE_DIRS})

find_package(OpenVR REQUIRED)
if (VERBOSE_CMAKE_MESSAGES)
    message("OpenVR_INCLUDE_DIRS:${OpenVR_INCLUDE_DIRS}")
    message("OpenVR_LIBRARIES:${OpenVR_LIBRARIES}")
    message("OpenVR_LIBRARY:${OpenVR_LIBRARY_DBG}")
endif()

if(ESVR2_LOWLATENCYVIDEOLOADER)
    if (VERBOSE_CMAKE_MESSAGES)
        message("USE_LOWLATENCYVIDEOLOADER")
    endif()
    add_definitions(-DUSE_LOWLATENCYVIDEOLOADER)
endif()

if(ESVR2_BLACKMAGIC_VIDEOLOADER)
    SET(BLACKMAGIC_HOME ${ESVR2_BLACKMAGIC_HOME})
    MESSAGE("-- Looking for BlackMagic in BLACKMAGIC_HOME:${BLACKMAGIC_HOME}")
    if(NOT ${BLACKMAGIC_HOME} STREQUAL "BLACKMAGIC_NOT_FOUND")
        if (VERBOSE_CMAKE_MESSAGES)
            message("USE_BLACKMAGIC_VIDEOLOADER")
        endif()
        add_definitions(-DUSE_BLACKMAGICCAMERA)
        set(BLACKMAGIC_INCLUDE_DIRS ${BLACKMAGIC_HOME}/include)
        set(BLACKMAGIC_LIBRARIES ${BLACKMAGIC_HOME}/lib/libblackmagic.so )
    endif()
endif()

if(ESVR2_POINTCLOUD)
    add_definitions(-DUSE_POINTCLOUD)
endif()

if(ESVR2_DOUBLE_PRECISION)
    add_definitions(-DUSE_DOUBLE_PRECISION)
endif()

SET(ESVR2_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config)

SET(ESVR2_MEDIA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/media)
SET(ESVR2_MEDIA_DIR_DEST ${ESVR2_MEDIA_DIR})
configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/CMake/Templates/Resources.cfg.in"
        ${EXECUTABLE_OUTPUT_PATH}/Resources.cfg
)

macro( add_recursive dir retVal )
    file( GLOB_RECURSE ${retVal} ${dir}/*.h ${dir}/*.cpp ${dir}/*.c )
endmacro()

add_recursive( ${CMAKE_CURRENT_SOURCE_DIR}/src/esvr2 SOURCES_LIB )
add_recursive( ${CMAKE_CURRENT_SOURCE_DIR}/include/esvr2 HEADERS_LIB )

include_directories(
        ${OGRE_INCLUDE_DIRS}
        ${SDL2_INCLUDE_DIR}
        ${OpenVR_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${BLACKMAGIC_INCLUDE_DIRS}
        ${SerialPort_INCLUDE_DIRS}
)

link_libraries(stdc++fs)

SET(ESVR2_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/esvr2)
SET(ESVR2_LIB ${PROJECT_NAME})
SET(ESVR2_APPLICATION ${PROJECT_NAME}Application)

include_directories(
        "${ESVR2_INCLUDE_DIR}"
        "pivot_control_messages/include")

add_library(
    ${ESVR2_LIB} SHARED
    ${SOURCES_LIB} ${HEADERS_LIB} ${RESOURCES})

add_executable(
    ${ESVR2_APPLICATION}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Esvr2Main.cpp
)

target_link_libraries(
    ${ESVR2_LIB}
    ${OGRE_LIBRARIES}
    ${SDL2_LIBRARY}
    ${OpenVR_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${CMAKE_DL_LIBS}
    ${SerialPort_LIBRARIES}
    pivot_control_messages
    ${BLACKMAGIC_LIBRARIES}
)

target_link_libraries(
    ${ESVR2_APPLICATION}
    ${ESVR2_LIB}
)

SET(ESVR2_MEDIA_DIR_DEST ${CMAKE_INSTALL_PREFIX}/media)
configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/CMake/Templates/Resources.cfg.in"
        ${CMAKE_BINARY_DIR}/Resource/Resources.cfg
)

install( TARGETS ${ESVR2_LIB} ${ESVR2_APPLICATION}
        DESTINATION bin
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install( DIRECTORY ${ESVR2_INCLUDE_DIR}
        DESTINATION include)
install( DIRECTORY ${ESVR2_MEDIA_DIR}
        DESTINATION ${CMAKE_INSTALL_PREFIX})
install( FILES  ${CMAKE_BINARY_DIR}/Resource/Resources.cfg ${ESVR2_CONFIG_DIR}/general.yml
        DESTINATION bin)
