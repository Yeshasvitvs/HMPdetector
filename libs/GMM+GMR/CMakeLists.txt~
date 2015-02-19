
PROJECT(GMM+GMR)

cmake_minimum_required(VERSION 2.8)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(LIBRARY_OUTPUT_PATH ${GMM+GMR_SOURCE_DIR}/lib)

include_directories(include ${HMPdetector_SOURCE_DIR}/libs/GMM+GMR/include)



file(GLOB GMM+GMR_SOURCES RELATIVE ${GMM+GMR_SOURCE_DIR} src/*.cpp)

ADD_DEFINITIONS(-Wall)

ADD_LIBRARY(GMM+GMR ${GMM+GMR_SOURCES})

#link_directories(${libUtils_SOURCE_DIR}/Utils)



