#-------------------------------------------------------------------
# This file is part of the CMake build system for Ogre
#     (Object-oriented Graphics Rendering Engine)
# For the latest info, see http://www.ogre3d.org/
#
# The contents of this file are placed in the public domain. Feel
# free to make use of it in any way you like.
#-------------------------------------------------------------------

# - Try to find Ogre
# If you have multiple versions of Ogre installed, use the CMake or
# the environment variable Ogre_HOME to point to the path where the
# desired Ogre version can be found.
# By default this script will look for a dynamic Ogre build. If you
# need to link against static Ogre libraries, set the CMake variable
# Ogre_STATIC to TRUE.
#
# Once done, this will define
#
#  Ogre_FOUND - system has Ogre
#  Ogre_INCLUDE_DIRS - the Ogre include directories
#  Ogre_LIBRARIES - link these to use the Ogre core
#  Ogre_BINARY_REL - location of the main Ogre binary (win32 non-static only, release)
#  Ogre_BINARY_DBG - location of the main Ogre binaries (win32 non-static only, debug)
#
# Additionally this script searches for the following optional
# parts of the Ogre package:
#  Plugin_CgProgramManager, Plugin_ParticleFX,
#  RenderSystem_GL, RenderSystem_GL3Plus,
#  RenderSystem_GLES, RenderSystem_GLES2,
#  RenderSystem_Direct3D9, RenderSystem_Direct3D11
#  Paging, Terrain, Volume, Overlay
#
# For each of these components, the following variables are defined:
#
#  Ogre_${COMPONENT}_FOUND - ${COMPONENT} is available
#  Ogre_${COMPONENT}_INCLUDE_DIRS - additional include directories for ${COMPONENT}
#  Ogre_${COMPONENT}_LIBRARIES - link these to use ${COMPONENT}
#  Ogre_${COMPONENT}_BINARY_REL - location of the component binary (win32 non-static only, release)
#  Ogre_${COMPONENT}_BINARY_DBG - location of the component binary (win32 non-static only, debug)
#
# Finally, the following variables are defined:
#
#  Ogre_PLUGIN_DIR_REL - The directory where the release versions of
#       the Ogre plugins are located
#  Ogre_PLUGIN_DIR_DBG - The directory where the debug versions of
#       the Ogre plugins are located
#  Ogre_MEDIA_DIR - The directory where the Ogre sample media is
#       located, if available

include(FindPkgMacros)
include(PreprocessorUtils)
findpkg_begin(Ogre)


# Get path, convert backslashes as ${ENV_${var}}
getenv_path(Ogre_HOME)
getenv_path(Ogre_SDK)
getenv_path(Ogre_SOURCE)
getenv_path(Ogre_BUILD)
getenv_path(Ogre_DEPENDENCIES_DIR)
getenv_path(PROGRAMFILES)

# Determine whether to search for a dynamic or static build
if (Ogre_STATIC)
    set(Ogre_LIB_SUFFIX "Static")
else ()
    set(Ogre_LIB_SUFFIX "")
endif ()

if(APPLE AND NOT Ogre_STATIC)
    set(Ogre_LIBRARY_NAMES "Ogre${Ogre_LIB_SUFFIX}")
else()
    set(Ogre_LIBRARY_NAMES "OgreMain${Ogre_LIB_SUFFIX}")
endif()
get_debug_names(Ogre_LIBRARY_NAMES)

# construct search paths from environmental hints and
# OS specific guesses
if (WIN32)
    set(Ogre_PREFIX_GUESSES
            ${ENV_PROGRAMFILES}/Ogre
            C:/OgreSDK
            )
elseif (UNIX)
    set(Ogre_PREFIX_GUESSES
            /opt/ogre
            /opt/Ogre
            /usr/lib${LIB_SUFFIX}/ogre
            /usr/lib${LIB_SUFFIX}/Ogre
            /usr/local/lib${LIB_SUFFIX}/ogre
            /usr/local/lib${LIB_SUFFIX}/Ogre
            $ENV{HOME}/ogre
            $ENV{HOME}/Ogre
            )
    if (APPLE)
        set(Ogre_PREFIX_GUESSES
                ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE}
                ${Ogre_PREFIX_GUESSES}
                )
    endif ()
endif ()
set(Ogre_PREFIX_PATH
        ${Ogre_HOME} ${Ogre_SDK} ${ENV_Ogre_HOME} ${ENV_Ogre_SDK}
        ${Ogre_PREFIX_GUESSES}
        )
create_search_paths(Ogre)
# If both Ogre_BUILD and Ogre_SOURCE are set, prepare to find Ogre in a build dir
set(Ogre_PREFIX_SOURCE ${Ogre_SOURCE} ${ENV_Ogre_SOURCE})
set(Ogre_PREFIX_BUILD ${Ogre_BUILD} ${ENV_Ogre_BUILD})
set(Ogre_PREFIX_DEPENDENCIES_DIR ${Ogre_DEPENDENCIES_DIR} ${ENV_Ogre_DEPENDENCIES_DIR})
if (Ogre_PREFIX_SOURCE AND Ogre_PREFIX_BUILD)
    foreach(dir ${Ogre_PREFIX_SOURCE})
        set(Ogre_INC_SEARCH_PATH ${dir}/OgreMain/include ${dir}/Dependencies/include ${dir}/iOSDependencies/include ${dir}/AndroidDependencies/include ${Ogre_INC_SEARCH_PATH})
        set(Ogre_LIB_SEARCH_PATH ${dir}/lib ${dir}/Dependencies/lib ${dir}/iOSDependencies/lib ${dir}/AndroidDependencies/lib/${ANDROID_ABI} ${Ogre_LIB_SEARCH_PATH})
        set(Ogre_BIN_SEARCH_PATH ${dir}/Samples/Common/bin ${Ogre_BIN_SEARCH_PATH})
    endforeach(dir)
    foreach(dir ${Ogre_PREFIX_BUILD})
        set(Ogre_INC_SEARCH_PATH ${dir}/include ${Ogre_INC_SEARCH_PATH})
        if(APPLE AND NOT Ogre_BUILD_PLATFORM_APPLE_IOS)
            set(Ogre_LIB_SEARCH_PATH ${dir}/lib/macosx ${Ogre_LIB_SEARCH_PATH})
        else()
            set(Ogre_LIB_SEARCH_PATH ${dir}/lib ${Ogre_LIB_SEARCH_PATH})
        endif()

        if (Ogre_BUILD_PLATFORM_APPLE_IOS)
            set(Ogre_LIB_SEARCH_PATH ${dir}/lib/iphoneos ${dir}/lib/iphonesimulator ${Ogre_LIB_SEARCH_PATH})
        endif()

        set(Ogre_BIN_SEARCH_PATH ${dir}/bin ${Ogre_BIN_SEARCH_PATH})
        set(Ogre_BIN_SEARCH_PATH ${dir}/Samples/Common/bin ${Ogre_BIN_SEARCH_PATH})

        if(APPLE AND NOT Ogre_BUILD_PLATFORM_APPLE_IOS)
            set(Ogre_BIN_SEARCH_PATH ${dir}/bin/macosx ${Ogre_BIN_SEARCH_PATH})
        endif()
    endforeach(dir)

    if (Ogre_PREFIX_DEPENDENCIES_DIR)
        set(Ogre_INC_SEARCH_PATH ${Ogre_PREFIX_DEPENDENCIES_DIR}/include ${Ogre_INC_SEARCH_PATH})
        set(Ogre_LIB_SEARCH_PATH ${Ogre_PREFIX_DEPENDENCIES_DIR}/lib ${Ogre_LIB_SEARCH_PATH})
        set(Ogre_BIN_SEARCH_PATH ${Ogre_PREFIX_DEPENDENCIES_DIR}/bin ${Ogre_BIN_SEARCH_PATH})
    endif()
else()
    set(Ogre_PREFIX_SOURCE "NOTFOUND")
    set(Ogre_PREFIX_BUILD "NOTFOUND")
endif ()

# redo search if any of the environmental hints changed
set(Ogre_COMPONENTS Paging Terrain Volume Overlay
        Plugin_CgProgramManager Plugin_ParticleFX
        RenderSystem_Direct3D11 RenderSystem_Direct3D9 RenderSystem_GL RenderSystem_GL3Plus RenderSystem_GLES RenderSystem_GLES2)
set(Ogre_RESET_VARS
        Ogre_CONFIG_INCLUDE_DIR Ogre_INCLUDE_DIR
        Ogre_FRAMEWORK_INCLUDES Ogre_FRAMEWORK_PATH Ogre_LIBRARY_FWK Ogre_LIBRARY_REL Ogre_LIBRARY_DBG
        Ogre_PLUGIN_DIR_DBG Ogre_PLUGIN_DIR_REL Ogre_MEDIA_DIR)
foreach (comp ${Ogre_COMPONENTS})
    set(Ogre_RESET_VARS ${Ogre_RESET_VARS}
            Ogre_${comp}_INCLUDE_DIR Ogre_${comp}_LIBRARY_FWK
            Ogre_${comp}_LIBRARY_DBG Ogre_${comp}_LIBRARY_REL
            )
endforeach (comp)
set(Ogre_PREFIX_WATCH ${Ogre_PREFIX_PATH} ${Ogre_PREFIX_SOURCE} ${Ogre_PREFIX_BUILD})
clear_if_changed(Ogre_PREFIX_WATCH ${Ogre_RESET_VARS})

if(NOT Ogre_STATIC)
    # try to locate Ogre via pkg-config
    use_pkgconfig(Ogre_PKGC "Ogre${Ogre_LIB_SUFFIX}")

    # Set the framework search path for OS X
    set(Ogre_FRAMEWORK_SEARCH_PATH
            ${CMAKE_FRAMEWORK_PATH}
            ~/Library/Frameworks
            /Library/Frameworks
            /System/Library/Frameworks
            /Network/Library/Frameworks
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/macosx/Release
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/macosx/Debug
            ${CMAKE_CURRENT_SOURCE_DIR}/lib/${CMAKE_BUILD_TYPE}
            )
else()
    set(Ogre_LIBRARY_FWK "")
endif()

# locate Ogre include files
find_path(Ogre_CONFIG_INCLUDE_DIR NAMES OgreBuildSettings.h HINTS ${Ogre_INC_SEARCH_PATH} ${Ogre_FRAMEWORK_INCLUDES} ${Ogre_PKGC_INCLUDE_DIRS} PATH_SUFFIXES "Ogre")
find_path(Ogre_INCLUDE_DIR NAMES OgreRoot.h HINTS ${Ogre_CONFIG_INCLUDE_DIR} ${Ogre_INC_SEARCH_PATH} ${Ogre_FRAMEWORK_INCLUDES} ${Ogre_PKGC_INCLUDE_DIRS} PATH_SUFFIXES "Ogre")
set(Ogre_INCOMPATIBLE FALSE)

if (Ogre_INCLUDE_DIR)
    if (NOT Ogre_CONFIG_INCLUDE_DIR)
        set(Ogre_CONFIG_INCLUDE_DIR ${Ogre_INCLUDE_DIR})
    endif ()
    # determine Ogre version
    file(READ ${Ogre_INCLUDE_DIR}/OgrePrerequisites.h Ogre_TEMP_VERSION_CONTENT)
    get_preprocessor_entry(Ogre_TEMP_VERSION_CONTENT Ogre_VERSION_MAJOR Ogre_VERSION_MAJOR)
    get_preprocessor_entry(Ogre_TEMP_VERSION_CONTENT Ogre_VERSION_MINOR Ogre_VERSION_MINOR)
    get_preprocessor_entry(Ogre_TEMP_VERSION_CONTENT Ogre_VERSION_PATCH Ogre_VERSION_PATCH)
    get_preprocessor_entry(Ogre_TEMP_VERSION_CONTENT Ogre_VERSION_NAME Ogre_VERSION_NAME)
    set(Ogre_VERSION "${Ogre_VERSION_MAJOR}.${Ogre_VERSION_MINOR}.${Ogre_VERSION_PATCH}")
    pkg_message(Ogre "Found Ogre ${Ogre_VERSION_NAME} (${Ogre_VERSION})")

    # determine configuration settings
    set(Ogre_CONFIG_HEADERS
            ${Ogre_CONFIG_INCLUDE_DIR}/OgreBuildSettings.h
            ${Ogre_CONFIG_INCLUDE_DIR}/OgreConfig.h
            )
    foreach(CFG_FILE ${Ogre_CONFIG_HEADERS})
        if (EXISTS ${CFG_FILE})
            set(Ogre_CONFIG_HEADER ${CFG_FILE})
            break()
        endif()
    endforeach()
    if (Ogre_CONFIG_HEADER)
        file(READ ${Ogre_CONFIG_HEADER} Ogre_TEMP_CONFIG_CONTENT)
        has_preprocessor_entry(Ogre_TEMP_CONFIG_CONTENT Ogre_STATIC_LIB Ogre_CONFIG_STATIC)
        get_preprocessor_entry(Ogre_TEMP_CONFIG_CONTENT Ogre_THREAD_SUPPORT Ogre_CONFIG_THREADS)
        get_preprocessor_entry(Ogre_TEMP_CONFIG_CONTENT Ogre_THREAD_PROVIDER Ogre_CONFIG_THREAD_PROVIDER)
        get_preprocessor_entry(Ogre_TEMP_CONFIG_CONTENT Ogre_NO_FREEIMAGE Ogre_CONFIG_FREEIMAGE)
        if (Ogre_CONFIG_STATIC AND Ogre_STATIC)
        elseif (Ogre_CONFIG_STATIC OR Ogre_STATIC)
            pkg_message(Ogre "Build type (static, dynamic) does not match the requested one.")
            set(Ogre_INCOMPATIBLE TRUE)
        endif ()
    else ()
        pkg_message(Ogre "Could not determine Ogre build configuration.")
        set(Ogre_INCOMPATIBLE TRUE)
    endif ()
else ()
    set(Ogre_INCOMPATIBLE FALSE)
endif ()

find_library(Ogre_LIBRARY_REL NAMES ${Ogre_LIBRARY_NAMES} HINTS ${Ogre_LIB_SEARCH_PATH} ${Ogre_PKGC_LIBRARY_DIRS} ${Ogre_FRAMEWORK_SEARCH_PATH} PATH_SUFFIXES "" "Release" "RelWithDebInfo" "MinSizeRel")
find_library(Ogre_LIBRARY_DBG NAMES ${Ogre_LIBRARY_NAMES_DBG} HINTS ${Ogre_LIB_SEARCH_PATH} ${Ogre_PKGC_LIBRARY_DIRS} ${Ogre_FRAMEWORK_SEARCH_PATH} PATH_SUFFIXES "" "Debug")

make_library_set(Ogre_LIBRARY)

if (Ogre_INCOMPATIBLE)
    set(Ogre_LIBRARY "NOTFOUND")
endif ()

if("${Ogre_FRAMEWORK_INCLUDES}" STREQUAL NOTFOUND)
    unset(Ogre_FRAMEWORK_INCLUDES CACHE)
endif()
set(Ogre_INCLUDE_DIR ${Ogre_CONFIG_INCLUDE_DIR} ${Ogre_INCLUDE_DIR} ${Ogre_FRAMEWORK_INCLUDES})
list(REMOVE_DUPLICATES Ogre_INCLUDE_DIR)
findpkg_finish(Ogre)
add_parent_dir(Ogre_INCLUDE_DIRS Ogre_INCLUDE_DIR)
if (Ogre_SOURCE)
    # If working from source rather than SDK, add samples include
    set(Ogre_INCLUDE_DIRS ${Ogre_INCLUDE_DIRS} "${Ogre_SOURCE}/Samples/Common/include")
endif()

mark_as_advanced(Ogre_CONFIG_INCLUDE_DIR Ogre_MEDIA_DIR Ogre_PLUGIN_DIR_REL Ogre_PLUGIN_DIR_DBG)

if (NOT Ogre_FOUND)
    return()
endif ()


# look for required Ogre dependencies in case of static build and/or threading
if (Ogre_STATIC)
    set(Ogre_DEPS_FOUND TRUE)
    find_package(Cg QUIET)
    find_package(DirectX QUIET)
    find_package(FreeImage QUIET)
    find_package(Freetype QUIET)
    find_package(OpenGL QUIET)
    find_package(OpenGLES QUIET)
    find_package(OpenGLES2 QUIET)
    find_package(ZLIB QUIET)
    find_package(ZZip QUIET)
    if (UNIX AND NOT APPLE AND NOT ANDROID)
        find_package(X11 QUIET)
        find_library(XAW_LIBRARY NAMES Xaw Xaw7 PATHS ${DEP_LIB_SEARCH_DIR} ${X11_LIB_SEARCH_PATH})
        if (NOT XAW_LIBRARY OR NOT X11_Xt_FOUND)
            set(X11_FOUND FALSE)
        endif ()
    endif ()

    set(Ogre_LIBRARIES ${Ogre_LIBRARIES} ${ZZip_LIBRARIES} ${ZLIB_LIBRARIES} ${FreeImage_LIBRARIES} ${FREETYPE_LIBRARIES})

    if (APPLE AND NOT Ogre_BUILD_PLATFORM_APPLE_IOS AND NOT ANDROID)
        set(Ogre_LIBRARIES ${Ogre_LIBRARIES} ${X11_LIBRARIES} ${X11_Xt_LIBRARIES} ${XAW_LIBRARY} ${X11_Xrandr_LIB} ${Carbon_LIBRARIES} ${Cocoa_LIBRARIES})
    endif()

    if (NOT ZLIB_FOUND OR NOT ZZip_FOUND)
        set(Ogre_DEPS_FOUND FALSE)
    endif ()
    if (NOT FreeImage_FOUND AND NOT Ogre_CONFIG_FREEIMAGE)
        set(Ogre_DEPS_FOUND FALSE)
    endif ()
    if (NOT FREETYPE_FOUND)
        set(Ogre_DEPS_FOUND FALSE)
    endif ()
    if (UNIX AND NOT APPLE AND NOT ANDROID)
        if (NOT X11_FOUND)
            set(Ogre_DEPS_FOUND FALSE)
        endif ()
    endif ()
endif()
if (Ogre_CONFIG_THREADS)
    if (Ogre_CONFIG_THREAD_PROVIDER EQUAL 1)
        if (Ogre_STATIC)
            set(Boost_USE_STATIC_LIBS TRUE)
            if(Ogre_BUILD_PLATFORM_APPLE_IOS)
                set(Boost_USE_MULTITHREADED OFF)
            endif()
        endif()

        set(Ogre_BOOST_COMPONENTS thread date_time)
        find_package(Boost COMPONENTS ${Ogre_BOOST_COMPONENTS} QUIET)
        if(Boost_FOUND AND Boost_VERSION GREATER 104900)
            if(Boost_VERSION GREATER 105300)
                set(Ogre_BOOST_COMPONENTS thread date_time system atomic chrono)
            else()
                set(Ogre_BOOST_COMPONENTS thread date_time system chrono)
            endif()
        endif()

        find_package(Boost COMPONENTS ${Ogre_BOOST_COMPONENTS} QUIET)
        if (NOT Boost_THREAD_FOUND)
            set(Ogre_DEPS_FOUND FALSE)
        else ()
            set(Ogre_LIBRARIES ${Ogre_LIBRARIES} ${Boost_LIBRARIES})
            set(Ogre_INCLUDE_DIRS ${Ogre_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
        endif ()
    elseif (Ogre_CONFIG_THREAD_PROVIDER EQUAL 2)
        find_package(POCO QUIET)
        if (NOT POCO_FOUND)
            set(Ogre_DEPS_FOUND FALSE)
        else ()
            set(Ogre_LIBRARIES ${Ogre_LIBRARIES} ${POCO_LIBRARIES})
            set(Ogre_INCLUDE_DIRS ${Ogre_INCLUDE_DIRS} ${POCO_INCLUDE_DIRS})
        endif ()
    elseif (Ogre_CONFIG_THREAD_PROVIDER EQUAL 3)
        find_package(TBB QUIET)
        if (NOT TBB_FOUND)
            set(Ogre_DEPS_FOUND FALSE)
        else ()
            set(Ogre_LIBRARIES ${Ogre_LIBRARIES} ${TBB_LIBRARIES})
            set(Ogre_INCLUDE_DIRS ${Ogre_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS})
        endif ()
    endif ()
endif ()
if (Ogre_STATIC)
    if (NOT Ogre_DEPS_FOUND)
        pkg_message(Ogre "Could not find all required dependencies for the Ogre package.")
        set(Ogre_FOUND FALSE)
    endif ()
endif ()

if (NOT Ogre_FOUND)
    return()
endif ()


get_filename_component(Ogre_LIBRARY_DIR_REL "${Ogre_LIBRARY_REL}" PATH)
get_filename_component(Ogre_LIBRARY_DIR_DBG "${Ogre_LIBRARY_DBG}" PATH)
set(Ogre_LIBRARY_DIRS ${Ogre_LIBRARY_DIR_REL} ${Ogre_LIBRARY_DIR_DBG})

# find binaries
if (NOT Ogre_STATIC)
    if (WIN32)
        find_file(Ogre_BINARY_REL NAMES "OgreMain.dll" HINTS ${Ogre_BIN_SEARCH_PATH}
                PATH_SUFFIXES "" Release RelWithDebInfo MinSizeRel)
        find_file(Ogre_BINARY_DBG NAMES "OgreMain_d.dll" HINTS ${Ogre_BIN_SEARCH_PATH}
                PATH_SUFFIXES "" Debug )
    endif()
    mark_as_advanced(Ogre_BINARY_REL Ogre_BINARY_DBG)
endif()


#########################################################
# Find Ogre components
#########################################################

set(Ogre_COMPONENT_SEARCH_PATH_REL
        ${Ogre_LIBRARY_DIR_REL}/..
        ${Ogre_LIBRARY_DIR_REL}/../..
        ${Ogre_BIN_SEARCH_PATH}
        )
set(Ogre_COMPONENT_SEARCH_PATH_DBG
        ${Ogre_LIBRARY_DIR_DBG}/..
        ${Ogre_LIBRARY_DIR_DBG}/../..
        ${Ogre_BIN_SEARCH_PATH}
        )

macro(ogre_find_component COMPONENT HEADER PATH_HINTS)
    set(Ogre_${COMPONENT}_FIND_QUIETLY ${Ogre_FIND_QUIETLY})
    findpkg_begin(Ogre_${COMPONENT})
    find_path(Ogre_${COMPONENT}_INCLUDE_DIR NAMES ${HEADER} HINTS ${Ogre_INCLUDE_DIRS} ${Ogre_PREFIX_SOURCE} PATH_SUFFIXES ${PATH_HINTS} ${COMPONENT} Ogre/${COMPONENT} )
    set(Ogre_${COMPONENT}_LIBRARY_NAMES "Ogre${COMPONENT}${Ogre_LIB_SUFFIX}")
    get_debug_names(Ogre_${COMPONENT}_LIBRARY_NAMES)
    find_library(Ogre_${COMPONENT}_LIBRARY_REL NAMES ${Ogre_${COMPONENT}_LIBRARY_NAMES} HINTS ${Ogre_LIBRARY_DIR_REL} ${Ogre_FRAMEWORK_PATH} PATH_SUFFIXES "" "Release" "RelWithDebInfo" "MinSizeRel")
    find_library(Ogre_${COMPONENT}_LIBRARY_DBG NAMES ${Ogre_${COMPONENT}_LIBRARY_NAMES_DBG} HINTS ${Ogre_LIBRARY_DIR_DBG} ${Ogre_FRAMEWORK_PATH} PATH_SUFFIXES "" "Debug")
    make_library_set(Ogre_${COMPONENT}_LIBRARY)
    findpkg_finish(Ogre_${COMPONENT})
    if (Ogre_${COMPONENT}_FOUND)
        # find binaries
        if (NOT Ogre_STATIC)
            if (WIN32)
                find_file(Ogre_${COMPONENT}_BINARY_REL NAMES "Ogre${COMPONENT}.dll" HINTS ${Ogre_COMPONENT_SEARCH_PATH_REL} PATH_SUFFIXES "" bin bin/Release bin/RelWithDebInfo bin/MinSizeRel Release)
                find_file(Ogre_${COMPONENT}_BINARY_DBG NAMES "Ogre${COMPONENT}_d.dll" HINTS ${Ogre_COMPONENT_SEARCH_PATH_DBG} PATH_SUFFIXES "" bin bin/Debug Debug)
            endif()
            mark_as_advanced(Ogre_${COMPONENT}_BINARY_REL Ogre_${COMPONENT}_BINARY_DBG)
        endif()
    endif()
    unset(Ogre_${COMPONENT}_FIND_QUIETLY)
endmacro()

# look for Paging component
ogre_find_component(Paging OgrePaging.h "")
# look for Terrain component
ogre_find_component(Terrain OgreTerrain.h "")
# look for Property component
ogre_find_component(Property OgreProperty.h "")
# look for RTShaderSystem component
ogre_find_component(RTShaderSystem OgreRTShaderSystem.h "")
# look for Volume component
ogre_find_component(Volume OgreVolumePrerequisites.h "")
# look for Overlay component
ogre_find_component(Overlay OgreOverlaySystem.h "")
#look for HlmsPbs component
ogre_find_component(HlmsPbs OgreHlmsPbs.h Hlms/Pbs/)
#look for HlmsPbsMobile component
ogre_find_component(HlmsPbsMobile OgreHlmsPbsMobile.h Hlms/PbsMobile/)
#look for HlmsPbsMobile component
ogre_find_component(HlmsUnlit OgreHlmsUnlit.h Hlms/Unlit)
#look for HlmsUnlit component
ogre_find_component(HlmsUnlitMobile OgreHlmsUnlitMobile.h Hlms/UnlitMobile)

#########################################################
# Find Ogre plugins
#########################################################
macro(ogre_find_plugin PLUGIN HEADER)
    # On Unix, the plugins might have no prefix
    if (CMAKE_FIND_LIBRARY_PREFIXES)
        set(TMP_CMAKE_LIB_PREFIX ${CMAKE_FIND_LIBRARY_PREFIXES})
        set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
    endif()

    # strip RenderSystem_ or Plugin_ prefix from plugin name
    string(REPLACE "RenderSystem_" "" PLUGIN_TEMP ${PLUGIN})
    string(REPLACE "Plugin_" "" PLUGIN_NAME ${PLUGIN_TEMP})

    # header files for plugins are not usually needed, but find them anyway if they are present
    set(Ogre_PLUGIN_PATH_SUFFIXES
            PlugIns PlugIns/${PLUGIN_NAME} Plugins Plugins/${PLUGIN_NAME} ${PLUGIN}
            RenderSystems RenderSystems/${PLUGIN_NAME} ${ARGN})
    find_path(Ogre_${PLUGIN}_INCLUDE_DIR NAMES ${HEADER}
            HINTS ${Ogre_INCLUDE_DIRS} ${Ogre_PREFIX_SOURCE}
            PATH_SUFFIXES ${Ogre_PLUGIN_PATH_SUFFIXES})
    # find link libraries for plugins
    set(Ogre_${PLUGIN}_LIBRARY_NAMES "${PLUGIN}${Ogre_LIB_SUFFIX}")
    get_debug_names(Ogre_${PLUGIN}_LIBRARY_NAMES)
    set(Ogre_${PLUGIN}_LIBRARY_FWK ${Ogre_LIBRARY_FWK})
    find_library(Ogre_${PLUGIN}_LIBRARY_REL NAMES ${Ogre_${PLUGIN}_LIBRARY_NAMES}
            HINTS "${Ogre_BUILD}/lib" ${Ogre_LIBRARY_DIRS} ${Ogre_FRAMEWORK_PATH} PATH_SUFFIXES "" Ogre Ogre-${Ogre_VERSION} opt Release Release/opt RelWithDebInfo RelWithDebInfo/opt MinSizeRel MinSizeRel/opt)
    find_library(Ogre_${PLUGIN}_LIBRARY_DBG NAMES ${Ogre_${PLUGIN}_LIBRARY_NAMES_DBG}
            HINTS "${Ogre_BUILD}/lib" ${Ogre_LIBRARY_DIRS} ${Ogre_FRAMEWORK_PATH} PATH_SUFFIXES "" Ogre Ogre-${Ogre_VERSION} opt Debug Debug/opt)
    make_library_set(Ogre_${PLUGIN}_LIBRARY)

    if (Ogre_${PLUGIN}_LIBRARY OR Ogre_${PLUGIN}_INCLUDE_DIR)
        set(Ogre_${PLUGIN}_FOUND TRUE)
        if (Ogre_${PLUGIN}_INCLUDE_DIR)
            set(Ogre_${PLUGIN}_INCLUDE_DIRS ${Ogre_${PLUGIN}_INCLUDE_DIR})
        endif()
        set(Ogre_${PLUGIN}_LIBRARIES ${Ogre_${PLUGIN}_LIBRARY})
    endif ()

    mark_as_advanced(Ogre_${PLUGIN}_INCLUDE_DIR Ogre_${PLUGIN}_LIBRARY_REL Ogre_${PLUGIN}_LIBRARY_DBG Ogre_${PLUGIN}_LIBRARY_FWK)

    # look for plugin dirs
    if (Ogre_${PLUGIN}_FOUND)
        if (NOT Ogre_PLUGIN_DIR_REL OR NOT Ogre_PLUGIN_DIR_DBG)
            if (WIN32)
                set(Ogre_PLUGIN_SEARCH_PATH_REL
                        ${Ogre_LIBRARY_DIR_REL}/..
                        ${Ogre_LIBRARY_DIR_REL}/../..
                        ${Ogre_BIN_SEARCH_PATH}
                        )
                set(Ogre_PLUGIN_SEARCH_PATH_DBG
                        ${Ogre_LIBRARY_DIR_DBG}/..
                        ${Ogre_LIBRARY_DIR_DBG}/../..
                        ${Ogre_BIN_SEARCH_PATH}
                        )
                find_path(Ogre_PLUGIN_DIR_REL NAMES "${PLUGIN}.dll" HINTS ${Ogre_PLUGIN_SEARCH_PATH_REL}
                        PATH_SUFFIXES "" bin bin/Release bin/RelWithDebInfo bin/MinSizeRel Release)
                find_path(Ogre_PLUGIN_DIR_DBG NAMES "${PLUGIN}_d.dll" HINTS ${Ogre_PLUGIN_SEARCH_PATH_DBG}
                        PATH_SUFFIXES "" bin bin/Debug Debug)
            elseif (UNIX)
                get_filename_component(Ogre_PLUGIN_DIR_TMP ${Ogre_${PLUGIN}_LIBRARY_REL} PATH)
                set(Ogre_PLUGIN_DIR_REL ${Ogre_PLUGIN_DIR_TMP} CACHE STRING "Ogre plugin dir (release)" FORCE)
                get_filename_component(Ogre_PLUGIN_DIR_TMP ${Ogre_${PLUGIN}_LIBRARY_DBG} PATH)
                set(Ogre_PLUGIN_DIR_DBG ${Ogre_PLUGIN_DIR_TMP} CACHE STRING "Ogre plugin dir (debug)" FORCE)
            endif ()
        endif ()

        # find binaries
        if (NOT Ogre_STATIC)
            if (WIN32)
                find_file(Ogre_${PLUGIN}_REL NAMES "${PLUGIN}.dll" HINTS ${Ogre_PLUGIN_DIR_REL})
                find_file(Ogre_${PLUGIN}_DBG NAMES "${PLUGIN}_d.dll" HINTS ${Ogre_PLUGIN_DIR_DBG})
            endif()
            mark_as_advanced(Ogre_${PLUGIN}_REL Ogre_${PLUGIN}_DBG)
        endif()

    endif ()

    if (TMP_CMAKE_LIB_PREFIX)
        set(CMAKE_FIND_LIBRARY_PREFIXES ${TMP_CMAKE_LIB_PREFIX})
    endif ()
endmacro(ogre_find_plugin)

ogre_find_plugin(Plugin_CgProgramManager OgreCgProgram.h PlugIns/CgProgramManager/include)
ogre_find_plugin(Plugin_ParticleFX OgreParticleFXPrerequisites.h PlugIns/ParticleFX/include)
ogre_find_plugin(RenderSystem_GL OgreGLRenderSystem.h RenderSystems/GL/include)
ogre_find_plugin(RenderSystem_GL3Plus OgreGL3PlusRenderSystem.h RenderSystems/GL3Plus/include)
ogre_find_plugin(RenderSystem_GLES OgreGLESRenderSystem.h RenderSystems/GLES/include)
ogre_find_plugin(RenderSystem_GLES2 OgreGLES2RenderSystem.h RenderSystems/GLES2/include)
ogre_find_plugin(RenderSystem_Direct3D9 OgreD3D9RenderSystem.h RenderSystems/Direct3D9/include)
ogre_find_plugin(RenderSystem_Direct3D11 OgreD3D11RenderSystem.h RenderSystems/Direct3D11/include)

if (Ogre_STATIC)
    # check if dependencies for plugins are met
    if (NOT DirectX_FOUND)
        set(Ogre_RenderSystem_Direct3D9_FOUND FALSE)
    endif ()
    if (NOT DirectX_D3D11_FOUND)
        set(Ogre_RenderSystem_Direct3D11_FOUND FALSE)
    endif ()
    if (NOT OPENGL_FOUND)
        set(Ogre_RenderSystem_GL_FOUND FALSE)
    endif ()
    if (NOT OPENGL_FOUND)
        set(Ogre_RenderSystem_GL3Plus_FOUND FALSE)
    endif ()
    if (NOT OPENGLES_FOUND)
        set(Ogre_RenderSystem_GLES_FOUND FALSE)
    endif ()
    if (NOT OPENGLES2_FOUND)
        set(Ogre_RenderSystem_GLES2_FOUND FALSE)
    endif ()
    if (NOT Cg_FOUND)
        set(Ogre_Plugin_CgProgramManager_FOUND FALSE)
    endif ()

    set(Ogre_RenderSystem_Direct3D9_LIBRARIES ${Ogre_RenderSystem_Direct3D9_LIBRARIES}
            ${DirectX_LIBRARIES}
            )

    set(Ogre_RenderSystem_Direct3D11_LIBRARIES ${Ogre_RenderSystem_Direct3D11_LIBRARIES}
            ${DirectX_D3D11_LIBRARIES}
            )
    set(Ogre_RenderSystem_GL_LIBRARIES ${Ogre_RenderSystem_GL_LIBRARIES}
            ${OPENGL_LIBRARIES}
            )
    set(Ogre_RenderSystem_GL3Plus_LIBRARIES ${Ogre_RenderSystem_GL3Plus_LIBRARIES}
            ${OPENGL_LIBRARIES}
            )
    set(Ogre_RenderSystem_GLES_LIBRARIES ${Ogre_RenderSystem_GLES_LIBRARIES}
            ${OPENGLES_LIBRARIES}
            )
    set(Ogre_RenderSystem_GLES2_LIBRARIES ${Ogre_RenderSystem_GLES2_LIBRARIES}
            ${OPENGLES2_LIBRARIES}
            )
    set(Ogre_Plugin_CgProgramManager_LIBRARIES ${Ogre_Plugin_CgProgramManager_LIBRARIES}
            ${Cg_LIBRARIES}
            )
endif ()

# look for the media directory
set(Ogre_MEDIA_SEARCH_PATH
        ${Ogre_SOURCE}
        ${Ogre_LIBRARY_DIR_REL}/..
        ${Ogre_LIBRARY_DIR_DBG}/..
        ${Ogre_LIBRARY_DIR_REL}/../..
        ${Ogre_LIBRARY_DIR_DBG}/../..
        ${Ogre_PREFIX_SOURCE}
        )
set(Ogre_MEDIA_SEARCH_SUFFIX
        Samples/Media
        Media
        media
        share/Ogre/media
        share/Ogre/Media
        )

clear_if_changed(Ogre_PREFIX_WATCH Ogre_MEDIA_DIR)
find_path(Ogre_MEDIA_DIR NAMES packs/cubemapsJS.zip HINTS ${Ogre_MEDIA_SEARCH_PATH}
        PATHS ${Ogre_PREFIX_PATH} PATH_SUFFIXES ${Ogre_MEDIA_SEARCH_SUFFIX})
