TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        ConstVelocityModel.cpp \
        main.cpp

HEADERS += \
    CheckResidual.h \
    ConstVelocityModel.h \
    ExtendedKalmanFilter.h \
    MathExpressions.h

LIBS += -larmadillo
LIBS += -lblas
LIBS += -llapack
