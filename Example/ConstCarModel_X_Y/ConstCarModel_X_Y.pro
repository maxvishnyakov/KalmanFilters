TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        ConstCarModel_X_Y.cpp \
        Example_ConstCarModel_X_Y.cpp

HEADERS += \
    ../ConstCarModel_X_Y.h \
    ../MathExpressions.h

INCLUDEPATH += $$PWD/../../KalmanFilters

LIBS += -larmadillo
LIBS += -lblas
LIBS += -llapack
