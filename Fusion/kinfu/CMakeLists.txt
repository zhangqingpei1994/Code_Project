cmake_minimum_required(VERSION 2.8.8)
project(kfu C CXX)

include_directories(/home/zhang/桌面/kinfu_remake-master/kfusion/include)

find_package(freenect2 REQUIRED)
include_directories( ${freenect2_INCLUDE_DIR})

find_package(CUDA 8.0 REQUIRED)

find_package(OpenCV 3.2.0 REQUIRED)
find_package(freenect2 REQUIRED)
include_directories( ${freenect2_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})





find_package(PCL 1.8 REQUIRED)
get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()
foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#set(srcs *.cpp *.hpp)     #把当前文件夹下的源码列表（文件后缀匹配的那些文件）存到变量 srcs 中
add_executable(demo grab_kinect2.cpp grab_kinect2.h demo.cpp)
target_link_libraries(demo /home/zhang/桌面/build-kinfu_remake-master-Desktop_Qt_5_6_1_GCC_64bit-Release/kfusion/libkfusion11.so)
target_link_libraries(demo ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${freenect2_LIBRARIES} ${CUDA_LIBRARIES})


