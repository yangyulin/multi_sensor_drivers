cmake_minimum_required(VERSION 2.8.3)
project(microstrain_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(GLIB2 REQUIRED)
find_Package(GTHREAD2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
)


# Our includes
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
)

# Files that we need to build
add_executable(${PROJECT_NAME}_node
  src/micro/microstrain_comm.cpp
  src/libbot/ringbuf.c
  src/libbot/rotations.c
  src/libbot/serial.c
  src/libbot/timestamp.c
)

# What libraries we need
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
)
