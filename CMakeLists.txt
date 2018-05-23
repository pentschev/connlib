cmake_minimum_required(VERSION 3.0)

#project(connlib DESCRIPTION "library to connect points")
#project(connlib VERSION 1.0)
project(connlib)

include(GNUInstallDirs)

set(CONNLIB_HEADERS
    include/connlib/connlib.h
    include/connlib/types.h
    include/connlib/utils.h)
set(CONNLIB_SOURCES src/connlib.cpp src/utils.cpp)
set(CONNLIB_TOOLS tools/connlib_sample.cpp)

add_library(connlib SHARED ${CONNLIB_SOURCES})

if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING
		"Build type: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)


set(CONNLIB_VERSION_MAJOR 0)
set(CONNLIB_VERSION_MINOR 1)
set_target_properties(connlib PROPERTIES VERSION ${CONNLIB_VERSION_MAJOR}.${CONNLIB_VERSION_MINOR})
set_target_properties(connlib PROPERTIES SOVERSION ${CONNLIB_VERSION_MAJOR})
target_include_directories(connlib PRIVATE include)
target_include_directories(connlib PRIVATE src)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")

install(TARGETS connlib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

include_directories(include)

option(BUILD_TOOLS "Build sample tools" true)
add_executable(connlib_sample tools/connlib_sample)
target_link_libraries(connlib_sample LINK_PUBLIC connlib)

find_package(GTest)
if(GTEST_FOUND)
    option(BUILD_TESTS "Build tests" true)
    if (BUILD_TESTS)
        find_package(Threads REQUIRED)
        enable_testing()
        include_directories(${GTEST_INCLUDE_DIRS})
        add_executable(connlib_test tests/connlib_test)
        target_link_libraries(connlib_test LINK_PUBLIC connlib
            ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
        add_executable(utils_example tests/utils_example)
        target_link_libraries(utils_example LINK_PUBLIC connlib
            ${GTEST_BOTH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
    endif (BUILD_TESTS)
endif(GTEST_FOUND)
