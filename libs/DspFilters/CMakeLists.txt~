# full library
# add_library(DspFilters Bessel.cpp Bessel.h Biquad.cpp Biquad.h Butterworth.cpp Butterworth.h Cascade.cpp Cascade.h ChebyshevI.cpp ChebyshevI.h ChebyshevII.cpp ChebyshevII.h Common.h Custom.cpp Custom.h Design.cpp Design.h Documentation.cpp Dsp.h Elliptic.cpp Elliptic.h Filter.cpp Filter.h Layout.h Legendre.cpp Legendre.h MathSupplement.h Param.cpp Params.h PoleFilter.cpp PoleFilter.h RBJ.cpp RBJ.h RootFinder.cpp RootFinder.h SmoothedFilter.h State.cpp State.h Types.h Utilities.h)


PROJECT(DspFilters)

cmake_minimum_required(VERSION 2.8)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(LIBRARY_OUTPUT_PATH ${DspFilters_SOURCE_DIR}/lib)

include_directories(include ${HMPdetector_SOURCE_DIR}/libs/DspFilters/include)



file(GLOB DspFilters_SOURCES RELATIVE ${DspFilters_SOURCE_DIR} src/*.cpp)

ADD_DEFINITIONS(-Wall)

ADD_LIBRARY(DspFilters ${DspFilters_SOURCES})

#link_directories(${libUtils_SOURCE_DIR}/Utils)


