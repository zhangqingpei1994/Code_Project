
cmake_minimum_required(VERSION 3.0)

PROJECT(head_position_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

SET(SRC_LIST test.cpp)


set(OpenCV_DIR /usr/local/share/OpenCV)
find_package(OpenCV 3.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

#寻找Ceres库并添加它的头文件
find_package(  dlib REQUIRED )
include_directories(  ${dlib_INCLUDE_DIRS} )

#opengl只需要添加.so就行
set(TARGET_LIB
     "/usr/lib/x86_64-linux-gnu/libGL.so"
     "/usr/lib/x86_64-linux-gnu/libGLU.so"
     "/usr/lib/x86_64-linux-gnu/libglut.so"
     )

ADD_EXECUTABLE(head_position_test ${SRC_LIST})
target_link_libraries(head_position_test ${dlib_LIBRARIES} ${OpenCV_LIBS})

ADD_EXECUTABLE(glAnthropometric3DModel glAnthropometric3DModel.cpp)
target_link_libraries(glAnthropometric3DModel ${TARGET_LIB} ${OpenCV_LIBS})
