#  SerialPort_FOUND - system has SerialPort
#  SerialPort_INCLUDE_DIRS - the SerialPort include directories
#  SerialPort_LIBRARIES - link these to use SerialPort

include(FindPkgMacros)
findpkg_begin(SerialPort)

set(SerialPort_PREFIX_GUESSES
        /usr/lib${LIB_SUFFIX}
        /usr/local/lib${LIB_SUFFIX}
        )


# construct search paths
set(SerialPort_PREFIX_PATH
        ${SerialPort_HOME} ${ENV_SerialPort_HOME} ${ENV_SerialPort_SDK}
        ${SerialPort_PREFIX_GUESSES})

create_search_paths(SerialPort)

# redo search if prefix path changed
clear_if_changed(SerialPort_PREFIX_PATH
        SerialPort_LIBRARY_FWK
        SerialPort_LIBRARY_REL
        SerialPort_LIBRARY_DBG
        SerialPort_INCLUDE_DIR
        )

set(SerialPort_LIBRARY_NAMES serialport)
set(SerialPort_LIBRARY_NAMES_DBG ${SerialPort_LIBRARY_NAMES})

if(WIN32)
    set(SerialPort_BINARY_NAMES serialport.dll)
    set(SerialPort_BINARY_NAMES_DBG ${SerialPort_BINARY_NAMES})
else()
    set(SerialPort_BINARY_NAMES serialport)
    set(SerialPort_BINARY_NAMES_DBG ${SerialPort_BINARY_NAMES})
endif()

use_pkgconfig(SerialPort_PKGC SerialPort)

findpkg_framework(SerialPort)

find_path(SerialPort_INCLUDE_DIR NAMES libserialport.h HINTS ${SerialPort_FRAMEWORK_INCLUDES} ${SerialPort_INC_SEARCH_PATH} ${SerialPort_PKGC_INCLUDE_DIRS} PATH_SUFFIXES SerialPort serialport)
find_library(SerialPort_LIBRARY_REL NAMES ${SerialPort_LIBRARY_NAMES} HINTS ${SerialPort_LIB_SEARCH_PATH} ${SerialPort_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
find_library(SerialPort_LIBRARY_DBG NAMES ${SerialPort_LIBRARY_NAMES_DBG} HINTS ${SerialPort_LIB_SEARCH_PATH} ${SerialPort_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)
make_library_set(SerialPort_LIBRARY)

if(WIN32)
    set(SerialPort_BIN_SEARCH_PATH ${OGRE_DEPENDENCIES_DIR}/bin ${CMAKE_SOURCE_DIR}/Dependencies/bin ${OPENVR_HOME}/dll
            ${ENV_OPENVR_HOME}/dll ${ENV_OGRE_DEPENDENCIES_DIR}/bin
            ${OGRE_SOURCE}/Dependencies/bin ${ENV_OGRE_SOURCE}/Dependencies/bin
            ${OGRE_SDK}/bin ${ENV_OGRE_SDK}/bin
            ${OGRE_HOME}/bin ${ENV_OGRE_HOME}/bin)

    find_file(SerialPort_BINARY_REL NAMES ${SerialPort_BINARY_NAMES} HINTS ${SerialPort_BIN_SEARCH_PATH} ${SerialPort_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
    find_file(SerialPort_BINARY_DBG NAMES ${SerialPort_BINARY_NAMES} HINTS ${SerialPort_BIN_SEARCH_PATH} ${SerialPort_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)
else()
    find_library(SerialPort_BINARY_REL NAMES ${SerialPort_BINARY_NAMES} HINTS ${SerialPort_LIB_SEARCH_PATH} ${SerialPort_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" release relwithdebinfo minsizerel)
    find_library(SerialPort_BINARY_DBG NAMES ${SerialPort_BINARY_NAMES_DBG} HINTS ${SerialPort_LIB_SEARCH_PATH} ${SerialPort_PKGC_LIBRARY_DIRS} PATH_SUFFIXES "" debug)
endif()

make_library_set(SerialPort_BINARY)

findpkg_finish(SerialPort)
add_parent_dir(SerialPort_INCLUDE_DIRS SerialPort_INCLUDE_DIR)
