cmake_minimum_required(VERSION 3.15)

# project
project(timercpp LANGUAGES CXX)

# default build type: release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

# C++11 or later
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS  OFF)

# include
include_directories("include")

# thread
FIND_PACKAGE(Threads REQUIRED)

# build 
add_executable(example example/example.cpp)

TARGET_LINK_LIBRARIES(example ${CMAKE_THREAD_LIBS_INIT})


