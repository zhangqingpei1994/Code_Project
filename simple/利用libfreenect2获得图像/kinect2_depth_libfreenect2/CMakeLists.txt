cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project("My Project")

set(CMAKE_CXX_FLAGS "-std=c++11")

# Set cmake prefix path to enable cmake to find freenect2
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
find_package(freenect2 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories("/usr/include/libusb-1.0/")

INCLUDE_DIRECTORIES(
  ${freenect2_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(test src/test.cpp)
target_link_libraries(test ${freenect2_LIBRARIES} ${OpenCV_LIBS})
