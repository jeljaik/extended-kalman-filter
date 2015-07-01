# Locate MATRIX LIB install directory

# This module defines
# MATRIX_FOUND, is set to true


# variables
# ---------
IF (NOT __MATRIXWRAPPER_NEWMAT__)
  SET(__MATRIXWRAPPER_NEWMAT__ OFF CACHE BOOL "define for newmat")
  MARK_AS_ADVANCED(__MATRIXWRAPPER_NEWMAT__)
ENDIF (NOT __MATRIXWRAPPER_NEWMAT__)
SET(__MATRIXWRAPPER_NEWMAT__ OFF)

IF (NOT __MATRIXWRAPPER_LTI__)
  SET(__MATRIXWRAPPER_LTI__ OFF CACHE BOOL "define for lti")
  MARK_AS_ADVANCED(__MATRIXWRAPPER_LTI__)
ENDIF (NOT __MATRIXWRAPPER_LTI__)
SET(__MATRIXWRAPPER_LTI__ OFF)

IF (NOT __MATRIXWRAPPER_BOOST__)
  SET(__MATRIXWRAPPER_BOOST__ OFF CACHE BOOL "define for boost")
  MARK_AS_ADVANCED(__MATRIXWRAPPER_BOOST__)
ENDIF (NOT __MATRIXWRAPPER_BOOST__)
SET(__MATRIXWRAPPER_BOOST__ OFF)


# install path
# ------------
IF(NOT MATRIX_LIB)
  SET( MATRIX_LIB boost CACHE STRING "Which matrix library to use: lti, newmat or boost")
ENDIF(NOT MATRIX_LIB)
MESSAGE("Searching for matrix lib ${MATRIX_LIB}")


# find libs
# ---------
IF (MATRIX_LIB STREQUAL "newmat")
  FIND_LIBRARY(NEWMAT newmat )
  FIND_PATH(NEWMAT_FOUND newmat/include.h )
  IF ( NEWMAT AND NEWMAT_FOUND )
    MESSAGE("-- Looking for Newmat - found")
    SET( MATRIX_INCLUDE "${NEWMAT_FOUND}")
    SET( MATRIX_LIBS "${NEWMAT}") 
    MESSAGE( "-- Newmat includes ${MATRIX_INCLUDE}")
    MESSAGE( "-- Newmat libs     ${MATRIX_LIBS}")
    SET(__MATRIXWRAPPER_NEWMAT__ ON)	
  ELSE ( NEWMAT AND NEWMAT_FOUND )
    MESSAGE( FATAL_ERROR "Looking for Newmat - not found")
  ENDIF ( NEWMAT AND NEWMAT_FOUND )
ELSE (MATRIX_LIB STREQUAL "newmat")


IF (MATRIX_LIB STREQUAL "lti")
  FIND_LIBRARY(LTI ltir )
  FIND_PATH(LTI_FOUND ltilib/ltiMatrix.h )
  IF ( LTI AND LTI_FOUND )
    MESSAGE("-- Looking for Lti - found")
    SET( MATRIX_INCLUDE "${LTI_FOUND}")
    SET( MATRIX_LIBS "${LTI}") 
    MESSAGE( "-- Lti includes ${MATRIX_INCLUDE}")
    MESSAGE( "-- Lti libs     ${MATRIX_LIBS}")
    SET(__MATRIXWRAPPER_LTI__ ON)	
  ELSE ( LTI AND LTI_FOUND )
    MESSAGE(FATAL ERROR "Looking for Lti in - not found")
  ENDIF ( LTI AND LTI_FOUND )
ELSE (MATRIX_LIB STREQUAL "lti")


IF (MATRIX_LIB STREQUAL "boost")
  FIND_PATH(BOOST_FOUND boost/numeric/ublas/matrix.hpp )
  IF ( BOOST_FOUND )
    MESSAGE("-- Looking for Boost - found")
    SET( MATRIX_INCLUDE "${BOOST_FOUND}")
    SET( MATRIX_LIBS "") 
    MESSAGE( "-- Boost includes ${MATRIX_INCLUDE}")
    MESSAGE( "-- Boost libs     ${MATRIX_LIBS}")
    SET(__MATRIXWRAPPER_BOOST__ ON)	
  ELSE ( BOOST_FOUND )
    MESSAGE(FATAL ERROR "Looking for Boost in - not found")
  ENDIF ( BOOST_FOUND )
ELSE (MATRIX_LIB STREQUAL "boost")


MESSAGE( FATAL_ERROR "No valid matrix lib specified. Please choose lti, newmat or boost")

ENDIF (MATRIX_LIB STREQUAL "boost")
ENDIF (MATRIX_LIB STREQUAL "lti")
ENDIF (MATRIX_LIB STREQUAL "newmat")
