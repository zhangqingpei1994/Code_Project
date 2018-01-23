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
    capture.cpp

HEADERS  += mainwindow.h \
    capture.h

FORMS    += mainwindow.ui

QT           += network

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2\
                /usr/include/ni           \
                /usr/include/eigen3 \

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_calib3d.so \
        /usr/local/lib/libopencv_imgcodecs.so \
        /usr/lib/libOpenNI.so
