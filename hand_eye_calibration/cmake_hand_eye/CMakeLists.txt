
cmake_minimum_required(VERSION 3.0)

PROJECT(hand_eye_calibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(SRC_LIST src/main.cpp src/hand_eye_calibrator.cpp include/hand_eye_calibrator.h)

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

#寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

ADD_EXECUTABLE(hand_eye_calibration ${SRC_LIST})
target_link_libraries(hand_eye_calibration ${OpenCV_LIBS} ${CERES_LIBRARIES})
