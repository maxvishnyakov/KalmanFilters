TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        ConstCarModel_X.cpp \
        Example_ConstCarModel_X.cpp

HEADERS += \
    ConstCarModel_X.h \
    ../MathExpressions.h

INCLUDEPATH += $$PWD/../../KalmanFilters

LIBS += -larmadillo
LIBS += -lblas
LIBS += -llapack

