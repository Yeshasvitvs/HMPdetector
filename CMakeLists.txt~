cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_CXX_FLAGS "-g -Wall")

PROJECT(HMPdetector)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

add_subdirectory(libs/DspFilters)  
add_subdirectory(libs/GMM+GMR)

# find Threads - required by SerialStream
find_package(Threads REQUIRED)
find_package(Armadillo REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
#add_message_files(
#   FILES

#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
   #DEPENDENCIES
   #std_msgs  # Or other packages containing msgs
 #)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES vision
  CATKIN_DEPENDS  roscpp std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  libs/GMM+GMR/include
  libs/DspFilters/include
)

# use gmr (in 'libs/GMM+GMR' subfolder)
#include_directories("${PROJECT_SOURCE_DIR}/libs/GMM+GMR")
set(GMR_LIBS ${GMR_LIBS} GMM+GMR)

# use ChebyshevI (in 'libs/DspFilters' subfolder)
#include_directories("${PROJECT_SOURCE_DIR}/libs/DspFilters")
set(FILTER_LIBS ${FILTER_LIBS} DspFilters)

# find Boost (thread, date-time, system) - required by SerialStream
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
if(USE_STATIC_BOOST)
    set(Boost_USE_STATIC_LIBS FALSE)
endif()
set(BOOST_LIBS ${BOOST_LIBS} thread date_time system)
find_package(Boost REQUIRED COMPONENTS ${BOOST_LIBS})

## Declare a cpp library
# add_library(vision
#   src/${PROJECT_NAME}/vision.cpp
# )

## Declare a cpp executable
ADD_EXECUTABLE(HMPdetector src/HMPdetector.cpp src/classifier.cpp  src/creator.cpp  src/reasoner.cpp  src/utils.cpp  src/FallDetector.cpp libs/SerialStream.cpp )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(vision_node vision_generate_messages_cpp)

## Specify libraries to link a library or executable target against
TARGET_LINK_LIBRARIES(HMPdetector ${GMR_LIBS} ${FILTER_LIBS} ${ARMADILLO_LIBRARIES} -larmadillo)
TARGET_LINK_LIBRARIES(HMPdetector -lpeiskernel_mt -lpeiskernel -lpthread)
TARGET_LINK_LIBRARIES(HMPdetector ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT}) 
TARGET_LINK_LIBRARIES(HMPdetector ${catkin_LIBRARIES})  ##NOTE : Remember to link with catkin_LIBRARIES too


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
#install(TARGETS biclops biclops_node
 # ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 #)

## Mark cpp header files for installation
# install(DIRECTORY 
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 #)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_biclops.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

INSTALL(
  TARGETS HMPdetector
  RUNTIME DESTINATION /usr/local/bin
  LIBRARY DESTINATION /usr/local/lib
  ARCHIVE DESTINATION /usr/local/lib
)








