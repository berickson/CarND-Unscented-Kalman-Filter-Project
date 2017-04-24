project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/ukf.cpp
   src/main.cpp
   src/tools.cpp
   src/measurement.cpp)
set(INCLUDE_DIRECTORIES $(PROJECT_SOURCE_DIR))

add_executable(UnscentedKF ${sources})
