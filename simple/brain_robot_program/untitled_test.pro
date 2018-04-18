#-------------------------------------------------
#
# Project created by QtCreator 2017-07-23T21:35:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport
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
    Face_recognition/facehome/zhang/program_hebing/build/untitled_test: error while loading shared libraries: libARToolKitPlus.so: cannot open shared object file: No such file orecognition.h \
    force_sensor/ftusb.h \
    PD/force_control.h \
    pose_detect/cameraposition.h \
    UR5_kin/ur5_kin.h \
    qcustomplot.h



FORMS    += mainwindow.ui

QT           += network

INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2 \
               /usr/local/include/ARToolKitPlus

INCLUDEPATH+=/usr/include/eigen3/Eigen\
             /home/zhang/program_hebing/brain_robot_program/omd            #程序移植时这个要改


LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_imgproc.so\
        /usr/local/lib/libopencv_objdetect.so\
        /usr/local/lib/libopencv_ml.so\
        /usr/local/lib/libopencv_video.so\
        /usr/local/lib/libopencv_face.so\
        /usr/local/lib/libopencv_shape.so\
        /usr/local/lib/libopencv_videoio.so\
        /usr/local/lib/libopencv_imgcodecs.so\
        /usr/local/lib/libARToolKitPlus.so

         /home/zhang/program_hebing/brain_robot_program/lib/libOMD.so


LIBS += -lARToolKitPlus




INCLUDEPATH += $$PWD/
DEPENDPATH += $$PWD/

DISTFILES +=

unix:!macx: LIBS += -L$$PWD/lib/ -lOMD

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

RESOURCES += \
    resource_tms.qrc
