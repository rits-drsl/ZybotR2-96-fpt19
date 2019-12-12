cmake_minimum_required(VERSION 2.8)
project(fad)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#-- enable output compile_command.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#-- cmake -DCMAKE_BUILD_TYPE=debug ..
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g -MMD -Wall -Wextra -Winit-self")

#-- cmake ..
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++14 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  add_definitions(-DCOMPILEDWITHC14)
  message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCOMPILEDWITHC0X)
  message(STATUS "Using flag -std=c++0x.")
else()
  message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 2.4.3 QUIET)
  if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
  endif()
endif()

find_package(Threads REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# 自作ライブラリへのパス
set(LIBRARIES_DIR "${PROJECT_SOURCE_DIR}/../../lib")

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${LIBRARIES_DIR}/include/improc
  ${LIBRARIES_DIR}/include/planner
  ${LIBRARIES_DIR}/include/zynqpl
  ${LIBRARIES_DIR}/include/control
  ${LIBRARIES_DIR}/include/optor
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  )

link_directories(
  ${LIBRARIES_DIR}
  )

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/fad/Handler.cpp
  src/fad/RoutePlanner/RoutePlanner.cpp
  src/fad/StateEstimator/StateEstimator.cpp
  src/fad/StateEstimator/ParticleFilter/ParticleFilter.cpp
  src/fad/ConstraintGenerator/ConstraintGenerator.cpp
  src/fad/ConstraintGenerator/TrafficLightDetector/TrafficLightDetector.cpp
  src/fad/ConstraintGenerator/PedestrianDetector/PedestrianDetector.cpp
  src/fad/OdometryCalculator/WOCalculator/WOCalculator.cpp
  src/fad/OdometryCalculator/IOCalculator/IOCalculator.cpp
  src/fad/OdometryCalculator/VOCalculator/VOCalculator.cpp
  src/fad/OdometryCalculator/RoadSurfaceVOCalculator/RoadSurfaceVOCalculator.cpp
  src/fad/OdometryCalculator/PatternMatchingVOCalculator/PatternMatchingVOCalculator.cpp
  src/fad/OdometryCalculator/PatternMatchingVOCalculator/PatternMatcher/PatternMatcherBase.cpp
  src/fad/OdometryCalculator/PatternMatchingVOCalculator/PatternMatcher/AKAZEPatternMatcher/AKAZEPatternMatcher.cpp
  src/fad/OdometryCalculator/PatternMatchingVOCalculator/PatternMatcher/TMPatternMatcher/TMPatternMatcher.cpp
  src/fad/PathPlanner/PathPlanner.cpp
  src/fad/PathTracer/PathTracer.cpp
  src/fad/PathTracer/Tracer/TracerBase.cpp
  src/fad/PathTracer/Tracer/PurePursuit/PurePursuit.cpp
  src/fad/PathTracer/Tracer/SelfLocalizedPurePursuit/SelfLocalizedPurePursuit.cpp
  src/fad/PathTracer/Tracer/LQR/LQR.cpp
  src/fad/HWController/CmodController/CmodController.cpp
  src/fad/HWController/PcamController/PcamController.cpp
  src/fad/HWController/PcamController/PcamImageCorrector/PcamImageCorrector.cpp
  src/fad/HWController/WebcamController/WebcamController.cpp
  src/fad/HWController/OptorCamController/OptorCamController.cpp
  src/fad/HWController/OptorIMUController/OptorIMUController.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  Threads::Threads
  planner
  zynqpl
  improc
  control
  optor
  )
