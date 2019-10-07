cmake_minimum_required(VERSION 2.8.3)
project(kr_attitude_eskf)


# include boost
#find_package(Boost REQUIRED)
#link_directories(${Boost_LIBRARY_DIR})
#include_directories(${Boost_INCLUDE_DIR})

# include headers
include_directories(
  include
  include/kr_attitude_eskf
  .
)

# build shared library
add_library(krAttitudeESKF
  src/AttitudeESKF.cpp
)

set(RELEVANT_LIBRARIES
  krAttitudeESKF
)

# rigorous error checking
add_definitions("-Wall -DATTITUDE_ESKF_BUILD_MAG -O3 -std=c++11")

# build main filter
add_executable(${PROJECT_NAME}
  src/kr_attitude_eskf.cpp
)
target_link_libraries(${PROJECT_NAME} ${RELEVANT_LIBRARIES})

# ensures messages are generated before hand
#add_dependencies(${PROJECT_NAME}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
#)
