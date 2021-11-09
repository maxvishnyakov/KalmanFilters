TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    ConstCarModel_A_R.cpp \
    Example_ConstCarModel_A_R.cpp

HEADERS += \
    ../MathExpressions.h \
    ConstCarModel_A_R.h

INCLUDEPATH += $$PWD/../../KalmanFilters

LIBS += -larmadillo
LIBS += -lblas
LIBS += -llapack
LIBS += -lboost_program_options
