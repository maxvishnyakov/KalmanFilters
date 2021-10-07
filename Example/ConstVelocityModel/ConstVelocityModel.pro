TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ConstCarMode_A_R.cpp \
    Example_ConstCarModel_A_R.cpp

HEADERS += \
    ../MathExpressions.h \
    ConstCarMode_A_R.h

INCLUDEPATH += $$PWD/../../KalmanFilters

LIBS += -larmadillo
LIBS += -lblas
LIBS += -llapack
