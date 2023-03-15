#-------------------------------------------------
#
# Project created by QtCreator 2023-02-16T09:40:22
#
#-------------------------------------------------

QT       += core gui multimedia

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ScreenRecord
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

INCLUDEPATH += D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/include

LIBS += D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/lib/avcodec.lib  \
        D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/lib/avdevice.lib  \
        D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/lib/avfilter.lib  \
        D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/lib/avformat.lib  \
        D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/lib/avutil.lib  \
        D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/lib/swresample.lib  \
        D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/lib/postproc.lib  \
        D:/Code/C++/ffmpeg/ffmpeg-4.3.1-win32-dev/ffmpeg-4.3.1-win32-dev/lib/swscale.lib

#INCLUDEPATH += $$PWD/../FFmpeg431dev/include

#LIBS    +=  $$PWD/../FFmpeg431dev/lib/avcodec.lib \
#            $$PWD/../FFmpeg431dev/lib/avdevice.lib \
#            $$PWD/../FFmpeg431dev/lib/avfilter.lib \
#            $$PWD/../FFmpeg431dev/lib/avformat.lib \
#            $$PWD/../FFmpeg431dev/lib/avutil.lib \
#            $$PWD/../FFmpeg431dev/lib/postproc.lib \
#            $$PWD/../FFmpeg431dev/lib/swresample.lib \
#            $$PWD/../FFmpeg431dev/lib/swscale.lib

SOURCES += \
        main.cpp \
        widget.cpp \
    cscreenrecorder.cpp

HEADERS += \
        widget.h \
    cscreenrecorder.h

FORMS += \
        widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
