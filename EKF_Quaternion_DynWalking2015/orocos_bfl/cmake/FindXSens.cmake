# Locate XSens install directory
#
# This module defines:
# XSENS_FOUND
# XSENS_LINCLUDE_DIRS
# XSENS_LIBRARIES

IF (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET (SYS_ARCH 64)
ELSE (CMAKE_SIZEOF_VOID_P EQUAL 8)
    SET (SYS_ARCH 32)
ENDIF (CMAKE_SIZEOF_VOID_P EQUAL 8)

STRING(COMPARE EQUAL "Linux" ${CMAKE_SYSTEM_NAME} LINUX_FOUND)
IF (LINUX_FOUND)
    MESSAGE("FindXsens.cmake module running in a Linux machine. We can proceed")

    set(XSENS_FOUND FALSE)

    IF (SYS_ARCH EQUAL 64)
        MESSAGE("LIBRARIES FOR 64BIT OS")
        set(XSENS_POSSIBLE_LIBRARY_PATHS /usr/local/xsens/lib64)
    ELSE (SYS_ARCH EQUAL 64)
        SET(XSENS_POSSIBLE_LIBRARY_PATHS /usr/local/xsens/lib32)
    ENDIF(SYS_ARCH EQUAL 64)

    set(XSENS_POSSIBLE_INCLUDE_DIRS /usr/local/xsens/public/include)

    # Find Include dirs. In the original installation of XSens there are more
    # than one location for the header. Here I'm hinting to the one in which
    # also the xcommunication headers are found.

    find_path(XSENS_CACHE
                NAMES xcommunication/protocolhandler.h
                PATHS ${XSENS_POSSIBLE_INCLUDE_DIRS}
                DOC   "Location of the xsens and xcommunication headers")

    IF (XSENS_CACHE)
        MESSAGE("-- Looking for XSens headers - found!")
        SET( XSENS_INCLUDE_DIR "${XSENS_CACHE}")
        MESSAGE("-- XSens includes in: ${XSENS_INCLUDE_DIR}")
    ELSE (XSENS_CACHE)
        MESSAGE(FATAL_ERROR "XSens was not found in the default directory /usr/local. Make sure you have installed it there. Currently we're not supporting user directories for this installation")
    ENDIF (XSENS_CACHE)

    # Find libraries
    find_library(XSENS_DEVICEAPI_LIB
                NAMES xsensdeviceapi
                HINTS ${XSENS_POSSIBLE_LIBRARY_PATHS}
                DOC   "Location of the xsensdeviceapi library")
    message("XSENS_DEVICEAPI_LIB IS: " ${XSENS_DEVICEAPI_LIB})

    find_library(XSENS_XSTYPES_LIB
                NAMES xstypes
                HINTS ${XSENS_POSSIBLE_LIBRARY_PATHS}
                DOC   "Location of the xsenstypes library")
    message("XSENS_XSTYPES_LIB IS: " ${XSENS_XSTYPES_LIB})
    
#   THE TWO LIBRARIES PREVIOUSLY FOUND MUST BE PUT IN ONE SINGLE VARIABLE XSENS_LIBRARIES. APPENDED SOMEHOW!
    SET(XSENS_LIBRARIES ${XSENS_DEVICEAPI_LIB} ${XSENS_XSTYPES_LIB})

    IF (XSENS_LIBRARIES)
        MESSAGE("-- Looking for XSens Libraries - found!")
        SET(XSENS_LIBRARIES "${XSENS_LIBRARIES}")
        MESSAGE("-- XSens libs: ${XSENS_LIBRARIES}")
    ELSE (XSENS_LIBRARIES)
        MESSAGE(FATAL_ERROR "XSens was not found in the default directory /usr/local. Make sure you have installed it there. Currently we're not supporting user directories for this installation")
    ENDIF (XSENS_LIBRARIES)

    SET(XSENS_FOUND TRUE)
 ELSE (LINUX_FOUND)
    MESSAGE(FATAL_ERROR "FindXSens.cmake currently works only on Linux distros")
 ENDIF (LINUX_FOUND)