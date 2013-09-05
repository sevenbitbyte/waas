#-------------------------------------------------
#
# Project created by QtCreator 2013-09-03T18:03:58
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = waas_config
TEMPLATE = app

LIBS += -L /usr/local/lib -lola  -lolacommon -lprotobuf
INCLUDEPATH += /usr/local/include

SOURCES += main.cpp\
        mainwindow.cpp \
    ledrun.cpp \
    utils.cpp \
    olamanager.cpp \
    pixelmapper.cpp

HEADERS  += mainwindow.h \
    ledrun.h \
    utils.h \
    olamanager.h \
    pixelmapper.h

FORMS    += mainwindow.ui
