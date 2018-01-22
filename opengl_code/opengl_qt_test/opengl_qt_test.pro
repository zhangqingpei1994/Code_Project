QT += core
QT -= gui

LIBS = -lGL -lGLU -lglut

CONFIG += c++11

TARGET = opengl_qt_test
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp
