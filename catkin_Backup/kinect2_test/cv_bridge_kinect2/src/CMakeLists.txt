# 增加PCL库的依赖
FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") # use this in Ubuntu 16.04
# 增加opencv的依赖
FIND_PACKAGE( OpenCV REQUIRED )

# 添加头文件和库文件
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
#INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

# INCLUDE_DIRECTORIES( ${PROJECT_SOURSE_DIR}/include )

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


ADD_LIBRARY( slambase slamBase.cpp )
TARGET_LINK_LIBRARIES( slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} )

ADD_EXECUTABLE( joinPointCloud joinPointCloud.cpp)
TARGET_LINK_LIBRARIES( joinPointCloud
    slambase
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES} 
    ${catkin_LIBRARIES}
    ${dlib_LIBRARIES})
