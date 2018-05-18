#-------------------------------------------------
#
# Project created by QtCreator 2017-12-09T10:33:19
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = test_temp
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    grab_kinect2.cpp \
    points_3d_show.cpp \
    robot_arm.cpp \
    handeye_calibrate.cpp

HEADERS  += mainwindow.h \
    grab_kinect2.h \
    points_3d_show.h \
    robot_arm.h \
    base.h \
    handeye_calibrate.h

FORMS    += mainwindow.ui

QT           += network



INCLUDEPATH +=  /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2\
                /usr/include/eigen3 \
                /usr/include/eigen3/Eigen\
                /usr/include/vtk-6.2\
                /usr/include/boost

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_calib3d.so \
        /usr/local/lib/libopencv_imgcodecs.so \
        /usr/local/lib/libfreenect2.so \

LIBS += /usr/lib/x86_64-linux-gnu/libboost_*.so \
#LIBS += /usr/lib/x86_64-linux-gnu/libvtk*-6.2.so


###  点云库PCL的头文件
INCLUDEPATH += /usr/include/pcl-1.8\
LIBS        += /usr/local/lib/libpcl_*.so \
LIBS        += /usr/lib/x86_64-linux-gnu/libpcl_visualization.so\
               /usr/lib/x86_64-linux-gnu/libpcl_common.so\
               /usr/lib/x86_64-linux-gnu/libpcl_io.so\

####  Dlib
INCLUDEPATH += -L/usr/local/include/dlib
LIBS += /usr/local/lib/libdlib.so\

###   Dlib库要用的,必须加,不然出错
INCLUDEPATH += -L/usr/include/openblas
LIBS += /usr/lib/libopenblas.so\
