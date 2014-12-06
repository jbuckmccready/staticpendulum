TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

QMAKE_CXXFLAGS += -std=c++11
QMAKE_CXXFLAGS += -pthread
QMAKE_CXXFLAGS_RELEASE += -ffast-math
QMAKE_CXXFLAGS_RELEASE += -march=native
QMAKE_CXXFLAGS_RELEASE += -funroll-loops
QMAKE_CXXFLAGS_RELEASE -= -O2
QMAKE_CXXFLAGS_RELEASE += -O3

SOURCES += main.cpp \
    pendulummap.cpp \
    pendulumsystem.cpp \
    integrators/cashkarp54.cpp \
    lodepng/lodepng.cpp \
    readconfig.cpp

HEADERS += \
    pendulummap.hpp \
    pendulumsystem.hpp \
    integrators/cashkarp54.hpp \
    lodepng/lodepng.h \
    threadpool.hpp \
    readconfig.hpp

LIBS += -pthread \
