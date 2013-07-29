CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(vle.extension.mas CXX C)
ENABLE_TESTING()
INCLUDE( CTest )

##
## Modules
##
INCLUDE(CheckIncludeFileCXX)
INCLUDE(CheckIncludeFile)
INCLUDE(CheckLibraryExists)
INCLUDE(CMakeDetermineCCompiler)

##
## Build type
##
if (CMAKE_BUILD_TYPE STREQUAL "")
  SET(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build" FORCE)
endif ()
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH})

##
## GCC Flags
##
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	  # using Clang
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Weverything")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")
	if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
	else()
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
	endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
	    # using GCC
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -std=c++11")
	option(WITH_COVERAGE "use coverage build [default: off]" OFF)
	if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ggdb3")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3")
		if (WITH_COVERAGE)
			set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage")
			set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
			set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
			set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} --coverage")
			set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
		endif ()
	else()
		set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
	endif ()
endif()
	

##
## Find boost libs
##
SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE(Boost 1.47 COMPONENTS graph random REQUIRED)
FIND_PACKAGE(Boost 1.47 COMPONENTS unit_test_framework)

##
## Find vle package
set(VLE_DEBUG 0)
find_package(VLE REQUIRED)

##
## Add source directory
##
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(exp)
##
## Tests
##
OPTION(WITH_TEST "will build the test [default: ON]" ON)
IF (Boost_UNIT_TEST_FRAMEWORK_FOUND AND WITH_TEST)
  ADD_SUBDIRECTORY(test)
ENDIF (Boost_UNIT_TEST_FRAMEWORK_FOUND AND WITH_TEST)

IF (Boost_UNIT_TEST_FRAMEWORK_FOUND)
  SET(HAVE_UNITTESTFRAMEWORK 1 CACHE INTERNAL "" FORCE)
ENDIF (Boost_UNIT_TEST_FRAMEWORK_FOUND)

##
## Generating documentation with doxygen
##
OPTION(WITH_DOXYGEN "build the documentation with doxygen [default: off]" OFF)
if (WITH_DOXYGEN)
  FIND_PACKAGE(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
      "Doxygen is needed to build the documentation. Please install it correctly")
  endif()
    add_subdirectory(doxygen)
endif (WITH_DOXYGEN)

##
## Install files
##
INSTALL(FILES COPYING README.md
  DESTINATION .)
