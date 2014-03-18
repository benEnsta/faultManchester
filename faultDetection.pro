#-------------------------------------------------
#
# Project created by QtCreator 2013-07-18T14:52:51
#
#-------------------------------------------------

QT       += core gui svg

TARGET = faultDetection
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    robot.cpp \
    tools/repere.cpp \
    interval/interval.cpp \
    interval/iboolean.cpp \
    interval/box.cpp \
    sivia.cpp

HEADERS  += mainwindow.h \
    robot.h \
    tools/repere.h \
    interval/interval.h \
    interval/iboolean.h \
    interval/box.h \
    sivia.h

FORMS    += mainwindow.ui
