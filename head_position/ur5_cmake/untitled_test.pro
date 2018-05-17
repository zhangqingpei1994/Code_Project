#-------------------------------------------------
#
# Project created by QtCreator 2017-07-23T21:35:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport network
TARGET = untitled_test
TEMPLATE = app
OBJECTS_DIR  = tmp

MOC_DIR      = tmp

SOURCES +=\
    bp_nn/bp.cpp\
    mythread/mythread.cpp\
    uart.cpp \
    tcpip_robot.cpp \
    mainwindow.cpp \
    main.cpp \
    robotState/UR5_state.cpp \
    Face_recognition/facemain.cpp \
    Face_recognition/facepreprocess.cpp \
    Face_recognition/facerecognition.cpp \
    force_sensor/ftusb.cpp \
    PD/force_control.cpp \
    pose_detect/cameraposition.cpp \
    UR5_kin/robotkin.cpp \
    qcustomplot.cpp




HEADERS  += \
    uart.h \
    tcpip_robot.h \
    mainwindow.h \
    omd/opto.h \
    omd/optodaq.h \
    omd/optopackage.h \
    omd/optopackage6d.h \
    omd/optoports.h \
    omd/sensorconfig.h \
    bp_nn/bp.h\
    mythread/mythread.h \
    robotState/UR5_state.h \
    Face_recognition/facemain.h \
    Face_recognition/facepreprocess.h \
    Face_recognition/facerecognition.h \
    force_sensor/ftusb.h \
    PD/force_control.h \
    pose_detect/cameraposition.h \
    UR5_kin/ur5_kin.h \
    qcustomplot.h



FORMS    += mainwindow.ui

QT           += network

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv \
                /usr/local/include/opencv2
INCLUDEPATH+=/home/linzc/QT-workspace/untitled_test_2/lib/eigen-eigen-67e894c6cd8f/Eigen\
             /home/linzc/QT/2-4/untitled_test2/omd

LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so\
       /usr/local/lib/libopencv_contrib.so\
       /usr/local/lib/libopencv_objdetect.so\
       /usr/local/lib/libopencv_ml.so
LIBS += -lARToolKitPlus

RESOURCES += \
    robot_resource.qrc



INCLUDEPATH += $$PWD/
DEPENDPATH += $$PWD/

DISTFILES +=

unix:!macx: LIBS += -L$$PWD/../lib/LINUX_API_V1.5.1/lib/ -lOMD

#INCLUDEPATH += $$PWD/../lib/LINUX_API_V1.5.1/include
#DEPENDPATH += $$PWD/../lib/LINUX_API_V1.5.1/include
