TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
        Example/ConstVelocityModel.cpp \
        Example/main.cpp

HEADERS += \
    KalmanFilters/CheckResidual.h \
    Example/ConstVelocityModel.h \
    KalmanFilters/ExtendedKalmanFilter.h \
    Example/MathExpressions.h \
    KalmanFilters/GaussPdf.h

LIBS += -larmadillo
LIBS += -lblas
LIBS += -llapack
