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

set( EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_BINARY_DIR}/bin" )
set( LIBRARY_OUTPUT_PATH  "${CMAKE_CURRENT_BINARY_DIR}/lib" )

SET(OGRE_HOME /opt/ogre)
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

#SUBDIRLIST( OGRE_INCLUDE_DIRS ${OGRE_HOME}/include)
SET(OGRE_INCLUDE_DIR ${OGRE_HOME}/include/OGRE)
SET(OGRE_INCLUDE_DIRS "${OGRE_INCLUDE_DIR}"
        "${OGRE_INCLUDE_DIR}/RenderSystems/GL3Plus"
        "${OGRE_INCLUDE_DIR}/Hlms/Unlit"
        "${OGRE_INCLUDE_DIR}/Hlms/Common"
        )
if(${CMAKE_BUILD_TYPE} MATCHES Debug)
    message("Debug")
    add_recursive_lib(${OGRE_HOME}/lib OGRE_LIBRARIES "_d")
else()
    message("Release")
    add_recursive_lib(${OGRE_HOME}/lib OGRE_LIBRARIES "")
endif()
SET(OGRE_MEDIA_DIR ${OGRE_HOME}/share/OGRE/Media)
message(OGRE_INCLUDE_DIRS:${OGRE_INCLUDE_DIRS})
message(OGRE_LIBRARIES:${OGRE_LIBRARIES})
SET(OGRE_VERSION 2.2.4)

#SET(Ogre_FOUND TRUE)
#SET(Ogre_INCLUDE_DIRS /opt )
#  Ogre_LIBRARIES - link these to use the Ogre core
#  Ogre_BINARY_REL - location of the main Ogre binary (win32 non-static only, release)
#  Ogre_BINARY_DBG - location of the main Ogre binaries (win32 non-static only, debug)


#find_package( Ogre PATHS ${Ogre_DIR} NO_DEFAULT_PATH REQUIRED)
if(NOT ${OGRE_VERSION} VERSION_GREATER 2.2.0)
    message(INFO "OGRE_INCLUDE_DIRS:${OGRE_INCLUDE_DIRS}")
    message(INFO "OGRE_LIBRARY_REL:${OGRE_LIBRARY_REL}")
    message(SEND_ERROR "Too Low Ogre Version (${OGRE_VERSION}) found")
else()
    message(INFO "OGRE_INCLUDE_DIRS:${OGRE_INCLUDE_DIRS}")
    message(INFO "OGRE_LIBRARY_REL:${OGRE_LIBRARY_REL}")
endif()

find_package( OpenCV REQUIRED )
message("OpenCV Libs: ${OpenCV_LIBS}")
message("OpenCV Include : ${OpenCV_INCLUDE_DIRS}")


#find_package(PkgConfig)

find_package( yaml-cpp REQUIRED )
Message(yaml-cpp_VERSION:${yaml-cpp_VERSION})
#pkg_check_modules(YAML_CPP yaml-cpp)
if(${yaml-cpp_VERSION} VERSION_GREATER 0.5)
    add_definitions(-DHAVE_NEW_YAMLCPP)
endif()
link_directories(${yaml-cpp_LIBRARY_DIRS})
include_directories(${yaml-cpp_INCLUDE_DIRS})

find_package(OpenVR REQUIRED)
message("OpenVR_INCLUDE_DIRS:${OpenVR_INCLUDE_DIRS}")
message("OpenVR_LIBRARIES:${OpenVR_LIBRARIES}")
message("OpenVR_LIBRARY:${OpenVR_LIBRARY_DBG}")

find_package(BlackMagic)
if(BlackMagic_FOUND)
    add_definitions(-DUSE_BLACKMAGICCAMERA)
endif()

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
        ${OpenVR_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${BLACKMAGIC_INCLUDE_DIRS} )

link_libraries(stdc++fs)

SET(ESVR2_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/esvr2)

include_directories(${ESVR2_INCLUDE_DIR})

add_library(
    ${PROJECT_NAME}_lib
    ${SOURCES_LIB} ${HEADERS_LIB} ${RESOURCES})

add_executable(
    ${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Esvr2Main.cpp
)

target_link_libraries(
    ${PROJECT_NAME}_lib
    ${OGRE_LIBRARIES}
    ${OpenVR_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES}
    ${CMAKE_DL_LIBS}
#    ${BLACKMAGIC_LIBRARIES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${PROJECT_NAME}_lib
)

SET(ESVR2_MEDIA_DIR_DEST ${CMAKE_INSTALL_PREFIX}/media)
configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/CMake/Templates/Resources.cfg.in"
        ${CMAKE_BINARY_DIR}/Resource/Resources.cfg
)

install( TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_lib
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib)
install( DIRECTORY ${ESVR2_INCLUDE_DIR}
        DESTINATION include)
install( DIRECTORY ${ESVR2_MEDIA_DIR}
        DESTINATION media)
install( FILES  ${CMAKE_BINARY_DIR}/Resource/Resources.cfg
        DESTINATION bin)
