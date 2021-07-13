#include "runningAverage.h"
#include "window.h"


GFilterRA::GFilterRA() {}

GFilterRA::GFilterRA(float coef, int interval) {
    _coef = coef;
    _filterInterval = interval;
}

GFilterRA::GFilterRA(float coef) {
    _coef = coef;
}

void GFilterRA::setCoef(float coef) {
    _coef = coef;
}
void GFilterRA::setStep(int interval) {
    _filterInterval = interval;
}

float GFilterRA::filteredTime(int value) {
    if (count - _filterTimer >= _filterInterval) {
        _filterTimer = count;
        filtered(value);
    }

    return _lastValue;
}

float GFilterRA::filteredTime(float value, int Timer) {
    if (Timer - _filterTimer >= _filterInterval) {
        _filterTimer = Timer;
        filtered(value);
    }
    return _lastValue;
}

float GFilterRA::filtered(int value) {
    _lastValue += (float)(value - _lastValue) * _coef;
    return _lastValue;
}

float GFilterRA::filtered(float value) {
    _lastValue += (float)(value - _lastValue) * _coef;
    return _lastValue;
}

//void GFilterRA::setCount(int value) {
//    count = value;
//}
