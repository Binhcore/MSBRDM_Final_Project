cmake_minimum_required(VERSION 2.8.3)
project(tum_ics_ur10_controller_tutorial)

SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  tum_ics_ur_robot_msgs
  tum_ics_ur_robot_lli
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  object_msgs
  ur_model
)

##############################################################################
# External Libs
##############################################################################
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(TMath REQUIRED)
add_definitions(${TMATH_DEFINITIONS})
include_directories(${TMATH_INCLUDE_DIRS})

find_package(RtThreads REQUIRED)
add_definitions(${RTTHREADS_DEFINITIONS})
include_directories(${RTTHREADS_INCLUDE_DIR})

find_package(Qt5Network)
add_definitions(${Qt5Network_DEFINITIONS})
include_directories(${Qt5Network_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS "${Qt5Network_EXECUTABLE_COMPILE_FLAGS}")

find_package(Qt5Widgets)
add_definitions(${Qt5Widgets_DEFINITIONS})
include_directories(${Qt5Widgets_INCLUDE_DIR})
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

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
  LIBRARIES ur10_control
  CATKIN_DEPENDS tum_ics_ur_robot_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversions visualization_msgs object_msgs tum_ics_ur_robot_lli ur_model
  DEPENDS ${DEPENDS_VAR}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -Wno-cpp -O3" )
set(CMAKE_BUILD_TYPE Release)

###########
## Build ##
###########

file(GLOB_RECURSE MY_CTRL_INC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
	include/*.h)

##############################################################################
# Sections Library. Here goes the src code of your controller
##############################################################################

add_library(ur10_control
   src/jointspace_controller.cpp
   src/cartesianspace_controller.cpp
   ${MY_CTRL_INC}
)
target_link_libraries(ur10_control
    ${catkin_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${tum_ics_ur_robot_lli_LIBRARIES}
    ${Qt5Network_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${RTTHREADS_LIBRARIES}
    ${TMATH_LIBRARIES}
)
add_dependencies(ur10_control
    ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS}
    ${sensor_msgs_EXPORTED_TARGETS}
    ${std_msgs_EXPORTED_TARGETS}
    ${visualization_msgs_EXPORTED_TARGETS}
)

##############################################################################
# Applications. Here goes your applications (using the controller class you defined in the library above)
##############################################################################

set(APP_TARGETS
    test_jointspace_controller
    test_cartesianspace_controller
)

foreach(target ${APP_TARGETS})
    add_executable(${target} src/Applications/${target}.cpp)
    target_link_libraries(${target}
        ur10_control
        ${catkin_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${tum_ics_ur_robot_lli_LIBRARIES}
        ${Qt5Network_LIBRARIES}
        ${Qt5Widgets_LIBRARIES}
        ${RTTHREADS_LIBRARIES}
        ${TMATH_LIBRARIES})
    add_dependencies(${target}
        ${tum_ics_ur_robot_msgs_EXPORTED_TARGETS}
        ${sensor_msgs_EXPORTED_TARGETS}
        ${std_msgs_EXPORTED_TARGETS}
        ${visualization_msgs_EXPORTED_TARGETS}
    )
endforeach()

##############################################################################
# Install
##############################################################################

######################################################################################

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

set(INS_TARGETS
    ur10_control
    ${APP_TARGETS}
)

install(TARGETS ${INS_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
